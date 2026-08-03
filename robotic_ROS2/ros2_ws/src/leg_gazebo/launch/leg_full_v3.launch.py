import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('leg_gazebo')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'leg_gazebo_v3.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'leg_world_v3.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['gzclient'],
                    output='screen'
                ),
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robotic_leg',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active',
                         'joint_state_broadcaster'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active',
                         'joint_trajectory_controller'],
                    output='screen'
                ),
            ]
        ),

        Node(
            package='leg_controller',
            executable='torque_bridge_node',
            name='torque_bridge_node',
            output='screen'
        ),

    ])
