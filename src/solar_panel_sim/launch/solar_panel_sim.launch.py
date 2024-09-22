import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='solar_panel_sim').find('solar_panel_sim')
    
    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'solar_panel.urdf.xacro')
    config_file = os.path.join(pkg_share, 'config', 'solar_panel_config.yaml')
    
    # Process the URDF file
    robot_description_content = Command([
        'xacro', ' ', urdf_file, ' ',
        'config_file:=', config_file
    ])
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare(package='gazebo_ros').find('gazebo_ros'),
            'launch', 'gazebo.launch.py')])
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'solar_panel'],
        output='screen'
    )
    
    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Run the controller
    solar_panel_controller = Node(
        package='solar_panel_sim',
        executable='solar_panel_controller',
        name='solar_panel_controller',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        solar_panel_controller
    ])