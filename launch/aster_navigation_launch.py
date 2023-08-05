import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='aster_navigation').find('aster_navigation')
    config_path = os.path.join(pkg_share, 'config/params.yaml')

    config = LaunchConfiguration('config')
    config_cmd = DeclareLaunchArgument(
        name='config',
        default_value=config_path,
        description='Absolute path to the params file'
    )

    aster_go_to_goal_node = Node(
        package="aster_navigation",
        executable="aster_go_to_goal",
        parameters=[config],
        remappings=[("/odom", "/odometry")]
    )

    ld = LaunchDescription()

    ld.add_action(config_cmd)
    ld.add_action(aster_go_to_goal_node)

    return ld
