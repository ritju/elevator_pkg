import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    # get params file path
    pkg_path = get_package_share_directory('elevator_pkg')
    params_file_name = 'config.yaml'
    params_file_path = os.path.join(pkg_path, 'params', params_file_name)
    
    launch_description = LaunchDescription()

    drive_into_elevator_action_server = Node(
        package='elevator_pkg',
        executable='drive_into_elevator_action_server',
        name='drive_into_elevator_action_server',
        parameters=[params_file_path],
        # respawn=True,
    )

    drive_outof_elevator_action_server = Node(
        package='elevator_pkg',
        executable='drive_outof_elevator_action_server',
        name='drive_outof_elevator_action_server',
        parameters=[params_file_path],
        # respawn=True,
    )

    move_to_goal_action_server = Node(
        package='elevator_pkg',
        executable='move_to_goal_action_server',
        name='move_to_goal_action_server',
        parameters=[params_file_path],
        # respawn=True,
    )

    launch_description.add_action(drive_into_elevator_action_server)
    launch_description.add_action(drive_outof_elevator_action_server)
    launch_description.add_action(move_to_goal_action_server)
    
    return launch_description














