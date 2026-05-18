#!/usr/bin/env python3
"""
ur_fake.launch.py  —  UR5e fake-hardware + MoveIt2 (simulation only)
════════════════════════════════════════════════════════════════════════
Launches two UR subsystems in the correct order:

  T=0 s   ur_control  (use_fake_hardware=true)
           └─ ros2_control fake interfaces
           └─ robot_state_publisher

  T=8 s   controller spawners  (explicit python3.10 to bypass shebang issue)
           └─ joint_state_broadcaster
           └─ scaled_joint_trajectory_controller

  T=12 s  ur_moveit  (use_fake_hardware=true)
           └─ move_group action server  →  /move_group
           └─ RViz with MoveIt plugin

No physical UR5e or network connection needed.

NOTE: The controller spawner binary uses #!/usr/bin/python3 which may point
to Python 3.13+ on newer Ubuntu systems.  We invoke it via python3.10
explicitly to stay compatible with ROS 2 Humble.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Path to the controller_manager spawner binary
SPAWNER = '/opt/ros/humble/lib/controller_manager/spawner'
PYTHON  = '/usr/bin/python3.10'


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────────── #
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type of UR robot (ur3, ur5, ur5e, ur10, ur10e, ur16e)',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz with MoveIt plugin',
    )

    ur_type     = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ── Paths to upstream launch files ───────────────────────────────── #
    ur_control_launch = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'),
        'launch',
        'ur_control.launch.py',
    ])
    ur_moveit_launch = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py',
    ])

    # ── 1. UR control plane (fake hardware, no spawner from upstream) ── #
    #   We pass controller_spawner_timeout=0 to make upstream spawners
    #   exit quickly, then we re-spawn with python3.10 below.
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch),
        launch_arguments={
            'ur_type':                   ur_type,
            'robot_ip':                  '0.0.0.0',   # ignored with fake hw
            'use_fake_hardware':         'true',
            'fake_sensor_commands':      'true',
            'launch_rviz':               'false',      # RViz launched by MoveIt
            'launch_dashboard_client':   'false',      # no real robot dashboard
            'controller_spawner_timeout': '1',         # let upstream spawners fail fast
        }.items(),
    )

    # ── 2. Re-spawn controllers with python3.10 (T=8 s) ─────────────── #
    #   By T=8 s the controller_manager is reliably up.
    #   We split into two sequential spawns (broadcaster first).
    spawn_jsb = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[PYTHON, SPAWNER,
                     '--controller-manager', '/controller_manager',
                     '--controller-manager-timeout', '15',
                     'joint_state_broadcaster'],
                output='screen',
            ),
        ],
    )

    spawn_jtc = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[PYTHON, SPAWNER,
                     '--controller-manager', '/controller_manager',
                     '--controller-manager-timeout', '15',
                     'scaled_joint_trajectory_controller'],
                output='screen',
            ),
        ],
    )

    # ── 3. MoveIt2 (T=14 s so controllers are confirmed active) ─────── #
    ur_moveit = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_moveit_launch),
                launch_arguments={
                    'ur_type':           ur_type,
                    'use_fake_hardware': 'true',
                    'launch_rviz':       launch_rviz,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        ur_type_arg,
        launch_rviz_arg,
        ur_control,
        spawn_jsb,
        spawn_jtc,
        ur_moveit,
    ])
