from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments (camera → robot-base transform) ─────────────── #
    cam_tx_arg = DeclareLaunchArgument('cam_tx', default_value='0.0',
        description='Camera X offset in robot base frame [m]')
    cam_ty_arg = DeclareLaunchArgument('cam_ty', default_value='0.0',
        description='Camera Y offset in robot base frame [m]')
    cam_tz_arg = DeclareLaunchArgument('cam_tz', default_value='0.0',
        description='Camera Z offset in robot base frame [m]')

    # ── Kinect camera node ────────────────────────────────────────────── #
    kinect_node = Node(
        package='kinect_picking',
        executable='kinect_node',
        name='kinect_node',
        output='screen',
    )

    # ── Shape / colour detection node ─────────────────────────────────── #
    detection_node = Node(
        package='kinect_picking',
        executable='detection_node',
        name='detection_node',
        output='screen',
    )

    # ── Depth extraction node ─────────────────────────────────────────── #
    depth_extractor_node = Node(
        package='kinect_picking',
        executable='depth_extractor_node',
        name='depth_extractor_node',
        output='screen',
    )

    # ── Camera-to-3D coordinate transform node ────────────────────────── #
    coord_transform_node = Node(
        package='kinect_picking',
        executable='coord_transform_node',
        name='coord_transform_node',
        output='screen',
    )

    # ── Voice command node ────────────────────────────────────────────── #
    voice_command_node = Node(
        package='kinect_picking',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
    )

    # ── Integrated robot commander node ───────────────────────────────── #
    #    Bridges vision pipeline → UR5e (arm + gripper).
    #    Tune cam_tx/ty/tz after physically measuring the Kinect position.
    robot_commander_node = Node(
        package='kinect_picking',
        executable='robot_commander_node',
        name='robot_commander_node',
        output='screen',
        parameters=[{
            # ── Camera → robot-base translation (update after calibration) ──
            'cam_tx': LaunchConfiguration('cam_tx'),
            'cam_ty': LaunchConfiguration('cam_ty'),
            'cam_tz': LaunchConfiguration('cam_tz'),
            # ── Pick geometry ───────────────────────────────────────────────
            'pre_grasp_offset': 0.10,   # m above object before descending
            'grasp_z_offset':   0.00,   # fine-tune grasp height
            # ── Gripper ─────────────────────────────────────────────────────
            'gripper_open':  0.085,     # Robotiq 2F fully open [m]
            'gripper_close': 0.000,     # Robotiq 2F fully closed [m]
            # ── Arm timing ──────────────────────────────────────────────────
            'move_duration': 5.0,       # s per joint-space move
            # ── MoveIt2 ─────────────────────────────────────────────────────
            'planning_group':  'ur_manipulator',
            'ee_link':         'tool0',
            'reference_frame': 'world',
            # ── End-effector orientation for picking (roll/pitch/yaw [rad]) ─
            'tool_roll':  1.5708,       # π/2 — horizontal approach
            'tool_pitch': 0.0,
            'tool_yaw':   0.0,
        }],
    )

    return LaunchDescription([
        cam_tx_arg,
        cam_ty_arg,
        cam_tz_arg,
        kinect_node,
        detection_node,
        depth_extractor_node,
        coord_transform_node,
        voice_command_node,
        robot_commander_node,
    ])
