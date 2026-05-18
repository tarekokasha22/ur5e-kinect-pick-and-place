#!/usr/bin/env python3
"""
kinect_picking_sim.launch.py  —  Full Vision Pipeline (Simulation Mode)
════════════════════════════════════════════════════════════════════════
Launches the complete Kinect + UR5e pick-and-place pipeline using:
  • Mock Kinect node  (synthetic RGB + depth — no hardware required)
  • Detection node    (HSV colour + shape detector)
  • Depth extractor   (median-patch depth reader)
  • Coord transform   (pixel → 3D in camera frame)
  • Robot commander   (voice-triggered MoveIt2 pick sequence)
  • [Optional] Voice command node (real microphone, or use CLI fallback)

Camera offsets are tuned so all 3 synthetic objects land within UR5e reach:
  cam_tx=0.5  cam_ty=0.1  cam_tz=-0.2
  (simulates kinect mounted 50 cm forward, 10 cm to the left, 20 cm below
   the robot base — adjust if you know your real mounting position)

ori_tolerance=0.5 rad  →  relaxed IK constraint for simulation
  (real-hardware launch keeps the default 0.10 rad for precision picks)

Usage
─────
  # Inside the kinect_ws (after sourcing):
  ros2 launch kinect_picking kinect_picking_sim.launch.py

  # Disable voice node (use CLI to send commands):
  ros2 launch kinect_picking kinect_picking_sim.launch.py use_voice:=false
  # Then send a command manually:
  ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'circle_1'"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────────── #
    use_voice_arg = DeclareLaunchArgument(
        'use_voice',
        default_value='false',
        description=(
            'Launch the microphone-based voice command node. '
            'Set false to control via: '
            'ros2 topic pub --once /voice_command std_msgs/msg/String '
            '"data: \'circle_1\'"'
        ),
    )
    cam_tx_arg = DeclareLaunchArgument(
        'cam_tx', default_value='0.5',
        description='Camera X offset in robot base frame [m]')
    cam_ty_arg = DeclareLaunchArgument(
        'cam_ty', default_value='0.1',
        description='Camera Y offset in robot base frame [m]')
    cam_tz_arg = DeclareLaunchArgument(
        'cam_tz', default_value='-0.2',
        description='Camera Z offset in robot base frame [m]')

    use_voice = LaunchConfiguration('use_voice')
    cam_tx    = LaunchConfiguration('cam_tx')
    cam_ty    = LaunchConfiguration('cam_ty')
    cam_tz    = LaunchConfiguration('cam_tz')

    # ── Mock Kinect (synthetic frames — no hardware) ──────────────────── #
    mock_kinect_node = Node(
        package='kinect_picking',
        executable='mock_kinect_node',
        name='kinect_node',                 # same name as real driver
        output='screen',
    )

    # ── Shape / colour detection ──────────────────────────────────────── #
    detection_node = Node(
        package='kinect_picking',
        executable='detection_node',
        name='detection_node',
        output='screen',
    )

    # ── Depth extraction ──────────────────────────────────────────────── #
    depth_extractor_node = Node(
        package='kinect_picking',
        executable='depth_extractor_node',
        name='depth_extractor_node',
        output='screen',
    )

    # ── Camera → 3D coordinate transform ─────────────────────────────── #
    coord_transform_node = Node(
        package='kinect_picking',
        executable='coord_transform_node',
        name='coord_transform_node',
        output='screen',
    )

    # ── Voice command node (optional — needs microphone) ─────────────── #
    voice_command_node = Node(
        package='kinect_picking',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        condition=IfCondition(use_voice),
    )

    # ── Robot commander (MoveIt2 Cartesian + joint moves) ────────────── #
    robot_commander_node = Node(
        package='kinect_picking',
        executable='robot_commander_node',
        name='robot_commander_node',
        output='screen',
        parameters=[{
            # ── Camera → robot-base translation (simulation-tuned) ──────
            'cam_tx': cam_tx,
            'cam_ty': cam_ty,
            'cam_tz': cam_tz,
            # ── Pick geometry ────────────────────────────────────────────
            'pre_grasp_offset': 0.10,   # m above object before descending
            'grasp_z_offset':   0.00,   # fine-tune grasp height
            # ── Gripper ──────────────────────────────────────────────────
            'gripper_open':  0.085,     # Robotiq 2F fully open [m]
            'gripper_close': 0.000,     # Robotiq 2F fully closed [m]
            # ── Arm timing ───────────────────────────────────────────────
            'move_duration': 5.0,       # s per joint-space move
            # ── MoveIt2 ──────────────────────────────────────────────────
            'planning_group':  'ur_manipulator',
            'ee_link':         'tool0',
            'reference_frame': 'world',
            # ── End-effector orientation ──────────────────────────────────
            'tool_roll':  1.5708,       # π/2 — horizontal approach
            'tool_pitch': 0.0,
            'tool_yaw':   0.0,
            # ── Relaxed IK tolerance for simulation ──────────────────────
            'ori_tolerance': 0.5,       # rad (real-HW default: 0.10 rad)
        }],
    )

    return LaunchDescription([
        use_voice_arg,
        cam_tx_arg,
        cam_ty_arg,
        cam_tz_arg,
        mock_kinect_node,
        detection_node,
        depth_extractor_node,
        coord_transform_node,
        voice_command_node,
        robot_commander_node,
    ])
