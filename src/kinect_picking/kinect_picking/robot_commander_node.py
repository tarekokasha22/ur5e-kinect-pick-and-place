#!/usr/bin/env python3
"""
Integrated Robot Commander Node
════════════════════════════════════════════════════════════════════
Bridges the kinect_picking vision pipeline with the UR5e robot arm.

Data flow:
  /detection/target_coords  (PointStamped, camera frame)
          │
          ▼  camera → robot-base transform  (tune cam_t* parameters)
          │
          ▼  MoveGroup action  →  MoveIt2  →  UR5e arm
          │
          ▼  GripperCommand action  →  Robotiq 2F gripper

Pick-and-place sequence
───────────────────────
  1. Home (joint move)
  2. Open gripper
  3. Pre-grasp  (Cartesian, pre_grasp_offset metres above object)
  4. Grasp      (Cartesian, at object)
  5. Close gripper
  6. Lift       (Cartesian, back to pre-grasp height)
  7. Drop zone  (joint move)
  8. Open gripper
  9. Home

ROS 2 parameters (override via launch or CLI)
─────────────────────────────────────────────
  cam_tx / cam_ty / cam_tz   : camera origin in robot-base frame [m]
  pre_grasp_offset           : height above object for pre-grasp [m]
  grasp_z_offset             : fine-tune grasp depth [m]
  gripper_open               : Robotiq position for open  [m]
  gripper_close              : Robotiq position for close [m]
  move_duration              : seconds per joint-space move
  planning_group             : MoveIt2 planning group name
  ee_link                    : end-effector link name
  reference_frame            : MoveIt2 reference frame
  tool_roll / tool_pitch / tool_yaw : EE orientation for picks [rad]
"""

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

# ── Joint names (UR5e) ────────────────────────────────────────────────────── #
JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# ── Pre-defined joint configurations ─────────────────────────────────────── #
#    Measure once from 'ros2 topic echo /joint_states' and update here.
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
DROP_JOINTS = [1.57, -1.30, 1.40, -1.57, -1.57, 0.0]


# ═══════════════════════════════════════════════════════════════════════════ #
class RobotCommanderNode(Node):
    """
    Voice-triggered pick-and-place using Kinect 3-D detections + UR5e.
    """

    def __init__(self):
        super().__init__('robot_commander_node')

        # ── Parameters ───────────────────────────────────────────────────── #
        self.declare_parameter('cam_tx', 0.0)
        self.declare_parameter('cam_ty', 0.0)
        self.declare_parameter('cam_tz', 0.0)
        self.declare_parameter('pre_grasp_offset', 0.10)   # m above object
        self.declare_parameter('grasp_z_offset',   0.00)   # fine-tune
        self.declare_parameter('gripper_open',     0.085)  # Robotiq fully open
        self.declare_parameter('gripper_close',    0.000)  # fully closed
        self.declare_parameter('move_duration',    5.0)    # s per joint move
        self.declare_parameter('planning_group',   'ur_manipulator')
        self.declare_parameter('ee_link',          'tool0')
        self.declare_parameter('reference_frame',  'world')
        self.declare_parameter('tool_roll',        1.5708) # π/2 → horizontal
        self.declare_parameter('tool_pitch',       0.0)
        self.declare_parameter('tool_yaw',         0.0)
        self.declare_parameter('ori_tolerance',    0.10)  # rad; set 0.5 in sim

        # ── Subscriptions ────────────────────────────────────────────────── #
        self.create_subscription(
            PointStamped, '/detection/target_coords',
            self._target_cb, 10)

        self.create_subscription(
            String, '/voice_command',
            self._voice_cb, 10)

        # ── Arm: joint trajectory publisher ─────────────────────────────── #
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10)

        # ── Gripper: Robotiq action client ───────────────────────────────── #
        self._GripperCommand = None
        self._gripper_client = None
        self._has_gripper = False
        self._init_gripper()

        # ── MoveGroup action client (MoveIt2 Cartesian moves) ────────────── #
        self._mg_client = ActionClient(self, MoveGroup, '/move_group')

        # ── State ────────────────────────────────────────────────────────── #
        self._is_executing = False
        self._last_label   = None   # prevent re-picking the same object
        self._lock         = threading.Lock()

        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('✅  Robot Commander (integrated) started')
        self.get_logger().info('   📥 /detection/target_coords')
        self.get_logger().info('   📥 /voice_command')
        self.get_logger().info('   📤 /scaled_joint_trajectory_controller/joint_trajectory')
        self.get_logger().info('   🎯 MoveGroup → /move_group')
        if self._has_gripper:
            self.get_logger().info('   ✋ Robotiq gripper → /robotiq_2f_urcap_adapter/gripper_command')
        else:
            self.get_logger().warn('   ⚠  Robotiq library not found — gripper in SIM mode')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('   Waiting for a voice command …')

    # ─────────────────────────────────────────────────────────────────────── #
    #  Gripper initialisation (optional import so node runs without ur_driver)
    # ─────────────────────────────────────────────────────────────────────── #

    def _init_gripper(self):
        try:
            from robotiq_2f_urcap_adapter.action import GripperCommand
            self._GripperCommand = GripperCommand
            self._gripper_client = ActionClient(
                self, GripperCommand,
                '/robotiq_2f_urcap_adapter/gripper_command')
            self._has_gripper = True
        except ImportError:
            self.get_logger().warn(
                'robotiq_2f_urcap_adapter not on PYTHONPATH. '
                'Source the ur_driver workspace to enable real gripper control.')

    # ─────────────────────────────────────────────────────────────────────── #
    #  ROS 2 callbacks                                                        #
    # ─────────────────────────────────────────────────────────────────────── #

    def _voice_cb(self, msg: String):
        """A new voice command resets the 'already-picked' guard."""
        with self._lock:
            self._last_label = None
        self.get_logger().info(
            f'🎙  Voice command received: "{msg.data}" → pick guard reset')

    def _target_cb(self, msg: PointStamped):
        """Called every frame a voice-selected target is visible."""
        label = msg.header.frame_id

        with self._lock:
            if self._is_executing:
                self.get_logger().debug(
                    f'Still executing — ignoring {label}')
                return
            if label == self._last_label:
                return          # same object already picked
            # Claim the slot
            self._is_executing = True

        self.get_logger().info(
            f'🎯  New pick target: {label}  '
            f'[cam X={msg.point.x:.3f} Y={msg.point.y:.3f} Z={msg.point.z:.3f} m]')

        thread = threading.Thread(
            target=self._pick_sequence, args=(msg,), daemon=True)
        thread.start()

    # ─────────────────────────────────────────────────────────────────────── #
    #  Camera → robot-base transform                                          #
    # ─────────────────────────────────────────────────────────────────────── #

    def _cam_to_robot(self, cx, cy, cz):
        """
        Simple translation-only transform.
        Update cam_tx/ty/tz after measuring the Kinect position in the
        robot base frame (use 'ros2 run tf2_ros tf2_echo world kinect_frame').
        """
        tx = self._param_f('cam_tx')
        ty = self._param_f('cam_ty')
        tz = self._param_f('cam_tz')
        return cx + tx, cy + ty, cz + tz

    # ─────────────────────────────────────────────────────────────────────── #
    #  Arm helpers                                                            #
    # ─────────────────────────────────────────────────────────────────────── #

    def _move_joints(self, positions, duration=None):
        """Send a joint-space trajectory to the arm."""
        if duration is None:
            duration = self._param_f('move_duration')

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(duration)
        msg.points = [pt]
        self._arm_pub.publish(msg)

        self.get_logger().info(
            f'🦾  Joint move → [{", ".join(f"{p:.2f}" for p in positions)}]')
        time.sleep(duration + 0.5)   # wait for execution

    def _move_cartesian(self, x, y, z, pos_tol=0.01, ori_tol=None):
        """
        Plan and execute a Cartesian goal via MoveGroup action.
        Returns True on success, False on failure / timeout.
        """
        if ori_tol is None:
            ori_tol = self._param_f('ori_tolerance')

        group = self._param_s('planning_group')
        ee    = self._param_s('ee_link')
        frame = self._param_s('reference_frame')
        roll  = self._param_f('tool_roll')
        pitch = self._param_f('tool_pitch')
        yaw   = self._param_f('tool_yaw')

        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)

        # Build PoseStamped target
        target = PoseStamped()
        target.header.frame_id = frame
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        # Position constraint (sphere around target)
        pc       = PositionConstraint()
        pc.header    = target.header
        pc.link_name = ee
        sp           = SolidPrimitive(type=SolidPrimitive.SPHERE,
                                      dimensions=[pos_tol])
        bv           = BoundingVolume(primitives=[sp],
                                      primitive_poses=[target.pose])
        pc.constraint_region = bv
        pc.weight = 1.0

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header      = target.header
        oc.link_name   = ee
        oc.orientation = target.pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        # Motion plan request
        req = MotionPlanRequest()
        req.group_name                  = group
        req.num_planning_attempts       = 10
        req.allowed_planning_time       = 10.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.goal_constraints = [Constraints(
            position_constraints=[pc],
            orientation_constraints=[oc])]

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan    = True

        self.get_logger().info(
            f'📐  Cartesian move → X={x:.3f} Y={y:.3f} Z={z:.3f} m')

        if not self._mg_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '❌  /move_group action server not reachable! '
                'Make sure ur_moveit.launch.py is running.')
            return False

        future = self._mg_client.send_goal_async(goal_msg)
        if not self._spin_until(future, timeout=15.0):
            self.get_logger().error('Timed out waiting for goal acceptance')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveGroup goal REJECTED')
            return False

        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future, timeout=30.0):
            self.get_logger().error('Timed out waiting for MoveGroup result')
            return False

        result = result_future.result()
        err_val = result.result.error_code.val
        if err_val != 1:          # moveit_msgs/MoveItErrorCodes SUCCESS = 1
            self.get_logger().error(
                f'MoveGroup returned error code {err_val}')
            return False

        self.get_logger().info('✔  Cartesian move succeeded')
        return True

    # ─────────────────────────────────────────────────────────────────────── #
    #  Gripper helper                                                         #
    # ─────────────────────────────────────────────────────────────────────── #

    def _gripper(self, pos: float):
        """
        Send a position goal to the Robotiq 2F gripper.
        pos = 0.085 → fully open, 0.0 → fully closed.
        Falls back to a log message when ur_driver is not sourced.
        """
        label = 'OPEN' if pos > 0.01 else 'CLOSE'

        if not self._has_gripper:
            self.get_logger().warn(f'[SIM] Gripper {label} (pos={pos:.3f})')
            time.sleep(1.0)
            return

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'❌  Gripper action server not available — cannot {label}')
            return

        goal = self._GripperCommand.Goal()
        goal.command.position   = pos
        goal.command.max_speed  = 0.1    # m/s  (0.02 – 0.15 valid range)
        goal.command.max_effort = 50.0   # N

        self.get_logger().info(f'✋  Gripper {label} (pos={pos:.3f})')
        future = self._gripper_client.send_goal_async(goal)
        self._spin_until(future, timeout=10.0)
        time.sleep(0.75)    # let jaws complete motion

    # ─────────────────────────────────────────────────────────────────────── #
    #  Full pick-and-place sequence                                           #
    # ─────────────────────────────────────────────────────────────────────── #

    def _pick_sequence(self, msg: PointStamped):
        label = msg.header.frame_id
        ok    = False

        try:
            # ── Convert to robot frame ────────────────────────────────── #
            rx, ry, rz = self._cam_to_robot(
                msg.point.x, msg.point.y, msg.point.z)
            self.get_logger().info(
                f'   Robot frame → X={rx:.3f} Y={ry:.3f} Z={rz:.3f} m')

            offset = self._param_f('pre_grasp_offset')
            dz     = self._param_f('grasp_z_offset')
            g_open = self._param_f('gripper_open')
            g_shut = self._param_f('gripper_close')

            # ── 1. Home ───────────────────────────────────────────────── #
            self.get_logger().info('── Step 1/9  Home ──────────────────')
            self._move_joints(HOME_JOINTS)

            # ── 2. Open gripper ───────────────────────────────────────── #
            self.get_logger().info('── Step 2/9  Open gripper ──────────')
            self._gripper(g_open)

            # ── 3. Pre-grasp (above object) ───────────────────────────── #
            self.get_logger().info('── Step 3/9  Pre-grasp ─────────────')
            if not self._move_cartesian(rx, ry, rz + offset):
                raise RuntimeError('Pre-grasp move failed')

            # ── 4. Grasp (lower to object) ────────────────────────────── #
            self.get_logger().info('── Step 4/9  Grasp ─────────────────')
            if not self._move_cartesian(rx, ry, rz + dz):
                raise RuntimeError('Grasp move failed')

            # ── 5. Close gripper ──────────────────────────────────────── #
            self.get_logger().info('── Step 5/9  Close gripper ─────────')
            self._gripper(g_shut)

            # ── 6. Lift ───────────────────────────────────────────────── #
            self.get_logger().info('── Step 6/9  Lift ──────────────────')
            if not self._move_cartesian(rx, ry, rz + offset):
                raise RuntimeError('Lift move failed')

            # ── 7. Drop zone ──────────────────────────────────────────── #
            self.get_logger().info('── Step 7/9  Drop zone ─────────────')
            self._move_joints(DROP_JOINTS)

            # ── 8. Open gripper ───────────────────────────────────────── #
            self.get_logger().info('── Step 8/9  Open gripper ──────────')
            self._gripper(g_open)

            # ── 9. Home ───────────────────────────────────────────────── #
            self.get_logger().info('── Step 9/9  Home ──────────────────')
            self._move_joints(HOME_JOINTS)

            ok = True
            self.get_logger().info(
                f'✅  Pick-and-place COMPLETE for {label}')

        except RuntimeError as exc:
            self.get_logger().error(f'❌  Sequence aborted: {exc}')
            self._safe_home()

        except Exception as exc:
            self.get_logger().error(
                f'❌  Unexpected error in pick sequence: {exc}')
            self._safe_home()

        finally:
            with self._lock:
                if ok:
                    self._last_label = label  # suppress re-pick until new voice cmd
                self._is_executing = False

    # ─────────────────────────────────────────────────────────────────────── #
    #  Safety recovery                                                        #
    # ─────────────────────────────────────────────────────────────────────── #

    def _safe_home(self):
        """Best-effort recovery: open gripper and return to home."""
        self.get_logger().warn('⚠  Running safety-home recovery …')
        try:
            self._gripper(self._param_f('gripper_open'))
            self._move_joints(HOME_JOINTS)
            self.get_logger().info('Recovery: arm is at home.')
        except Exception as exc:
            self.get_logger().error(f'Recovery also failed: {exc}')

    # ─────────────────────────────────────────────────────────────────────── #
    #  Utilities                                                              #
    # ─────────────────────────────────────────────────────────────────────── #

    def _spin_until(self, future, timeout=20.0):
        """Poll a future until done (background-thread friendly)."""
        t0 = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - t0 > timeout:
                return False
        return True

    def _param_f(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _param_s(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        """Convert roll-pitch-yaw [rad] to quaternion (x, y, z, w)."""
        cr, sr = np.cos(roll  / 2.0), np.sin(roll  / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        cy, sy = np.cos(yaw   / 2.0), np.sin(yaw   / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )


# ═══════════════════════════════════════════════════════════════════════════ #
def main(args=None):
    rclpy.init(args=args)
    node = RobotCommanderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
