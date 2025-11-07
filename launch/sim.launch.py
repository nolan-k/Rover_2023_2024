import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name='auto_bot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'auto_bot',
        ], 
        output='screen'
    )

   
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )


    delayed_ddc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_controller_spawner]
        )
    )
    delayed_jsb_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=spawn_entity,
             on_exit=[joint_state_broadcaster_spawner],
         )
    )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        #diff_drive_controller_spawner,
        #joint_state_broadcaster_spawner,
        delayed_ddc_spawner,
        delayed_jsb_spawner,
    ])
