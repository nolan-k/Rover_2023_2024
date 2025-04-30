from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

import os

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    pkg_name = 'rover2_control'  # <- update this to your package name
    urdf_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'rover2_control', 'drive_control.xacro'])
    controller_config = PathJoinSubstitution([FindPackageShare(pkg_name), 'rover2_control', 'odrive_drive_ros2_control.yaml'])

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="main",
        description="Ros2 Control Hardware Interface Type [main, sim]",
    )


    drive_ros2_controllers_path = os.path.join(
        get_package_share_directory("rover2_control"),
        "rover2_control",
        "odrive_drive_ros2_control.yaml",
    )

    #ros2_control_node Likely missing rover_description
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[drive_ros2_controllers_path],

    )
    # ros2_control Node
    ros2_control_node_2 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': Command([
            FindExecutable(name='xacro'), ' ', urdf_path
        ])}, controller_config],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command([
            FindExecutable(name='xacro'), ' ', urdf_path
        ])}]
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node"
    )

    # Joy Node to convert joystick input to velocities
    joy_to_drive_node = Node(
        package='rover2_control',  # Replace with your package name
        executable='joy_to_drive',  # This is the node you created above
        name='joy_to_drive',
        output='screen'
    )

    # Load joint_state_broadcaster after ros2_control_node is up
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    can_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['can_controller', "-c", "/controller_manager"],
        output='screen'
    )

    return LaunchDescription([
        ros2_control_node_2,
        ros2_control_node,
        robot_state_publisher,
        joy_node,
        joy_to_drive_node,
        joint_state_broadcaster_node,
        can_controller_node,       
    ])
