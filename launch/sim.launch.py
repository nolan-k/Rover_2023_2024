import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    ros2_control = LaunchConfiguration('ros2_control', default='true') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    localization = LaunchConfiguration('localization', default='false')

    package_name='auto_bot'
    pkg_share = get_package_share_directory(package_name)

    gazebo_params = os.path.join(pkg_share, 'config', 'gazebo_params.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'rgbd_nav2_params.yaml')
    rtabmap_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'depth_nav.rviz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share,'launch','rsp.launch.py')
        ]), 
        launch_arguments={
            'use_sim_time': 'true', 
            'ros2_control': ros2_control
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': f'--ros-args --params-file {gazebo_params}'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'auto_bot'], 
        output='screen'
    )

   
    diff_drive_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller'],
        #remappings=[('diff_drive_controller/odom', '/odom')],
        output='screen'
        # ros2 run topic_tools relay /diff_drive_controller/odom /odom
    )
    #delayed_ddc_spawner = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #        target_action=spawn_entity,
    #        on_exit=[diff_drive_controller_spawner]
    #    )
    #)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    #delayed_jsb_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    #)
    
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', 
        name='rtabmap',
        parameters=[rtabmap_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rtabmnap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry',
        name='rgbd_odometry',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/rgb/image", "/camera_depth/image_raw"),
            ("/depth/image", "/camera_depth/depth/image_raw"),
            ("/rgb/camera_info", "/camera_depth/camera_info")
        ],
        output='screen',
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','joystick.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params
        }.items()
    )

    spawn_controllers_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_controller_spawner]
        )
    )

    spawn_slam_nav_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[rtabmap, rtabmnap_odom, nav2]
        )
    )

    # turtlebot3_rgbd launch 
    parameters={
        'frame_id':'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth':True,
        'subscribe_rgb':True,
        'subscribe_scan':False,

        'use_action_for_goal':True,
        'Reg/Force3DoF':'true',
        'Grid/RayTracing':'true', # Fill empty space
        'Grid/3D':'false', # Use 2D occupancy
        'Grid/RangeMax':'10.0', # Max obstacle detection range (meters)
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw')]


    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz',
        parameters=[parameters],
        remappings=remappings,
        output='screen',
    )

    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    # Second, we segment the floor from the obstacles.
    # rtabmap_point_cloud = Node(
    #     package='rtabmap_util', executable='point_cloud_xyz', output='screen',
    #     parameters=[{'decimation': 2,
    #                  'max_depth': 10.0,
    #                  'voxel_size': 0.02}],
    #     remappings=[('depth/image', '/camera/depth/image_raw'),
    #                 ('depth/camera_info', '/camera/camera_info'),
    #                 ('cloud', '/camera/cloud')])
    #
    # rtabmap_obstacles = Node(
    #     package='rtabmap_util', executable='obstacles_detection', output='screen',
    #     parameters=[parameters],
    #     remappings=[('cloud', '/camera/cloud'),
    #                 ('obstacles', '/camera/obstacles'),
    #                 ('ground', '/camera/ground')])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch rtabmap in localization mode.'),

        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        joystick,
        spawn_controllers_after_entity,
        spawn_slam_nav_after_entity,
        #diff_drive_controller_spawner,
        #joint_state_broadcaster_spawner,
        #delayed_ddc_spawner,
        #delayed_jsb_spawner,
        #rtabmap,
        #rtabmap_slam,
        #rtabmap_localization,
        #rtabmap_viz,
        #rtabmap_point_cloud,
        #rtabmap_obstacles
    ])
