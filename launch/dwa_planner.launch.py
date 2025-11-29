from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='custom_dwa_planner').find('custom_dwa_planner')
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz'])

    return LaunchDescription([
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner_node',
            name='dwa_planner_node',
            output='screen',
            parameters=[{
                'goal_threshold': 0.15,
                'robot_radius': 0.22,
                'max_speed': 0.3,
                'predict_time': 1.5,
            }]
        )#,

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config],
        # )
    ])