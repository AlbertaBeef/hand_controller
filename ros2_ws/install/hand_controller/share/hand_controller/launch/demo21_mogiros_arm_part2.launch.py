import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_simple_arm = get_package_share_directory('bme_ros2_simple_arm')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_simple_arm)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    viewer1_name_arg = DeclareLaunchArgument(
        "viewer1_name",
        default_value="Hand Controller",
        description="Name of Image Viewer for hand_controller/annotations."
    )
    viewer2_name_arg = DeclareLaunchArgument(
        "viewer2_name",
        default_value="Gripper Camera",
        description="Name of Image Viewer for gripper mounted camera (in Gazebo)."
    )
    viewer3_name_arg = DeclareLaunchArgument(
        "viewer3_name",
        default_value="Table Camera",
        description="Name of Image Viewer for table mounted camera (in Gazebo)."
    )
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_arm.xacro',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )

    z_arg = DeclareLaunchArgument(
        'z', default_value='1.04',
        description='z coordinate of spawned robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw angle of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_bme_ros2_simple_arm,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('bme_ros2_simple_arm'),
        'config',
        'gz_bridge.yaml'
    )

    robot_controllers = PathJoinSubstitution(
        [
            get_package_share_directory('bme_ros2_simple_arm'),
            'config',
            'controller_position.yaml',
        ]
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_ros2_simple_arm, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Hand controller (most nodes are launch in *_part1.launch.py)
    hand_controller_viewer_node = Node(
        package='hand_controller',
        executable='usbcam_subscriber_node',
        name="hand_controller_annotations",
        parameters=[
            {"viewer_name":LaunchConfiguration("viewer1_name")}
        ],
        remappings=[
            ("image_raw", "hand_controller/image_annotated")
        ]
    ) 
    gripper_camera_viewer_node = Node(
        package='hand_controller',
        executable='usbcam_subscriber_node',
        name="gripper_camera_viewer",
        parameters=[
            {"viewer_name":LaunchConfiguration("viewer2_name")}
        ],
        remappings=[
            ("image_raw", "gripper_camera/image")
        ]
    ) 
    table_camera_viewer_node = Node(
        package='hand_controller',
        executable='usbcam_subscriber_node',
        name="table_camera_viewer",
        parameters=[
            {"viewer_name":LaunchConfiguration("viewer3_name")}
        ],
        remappings=[
            ("image_raw", "table_camera/image")
        ]
    ) 

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_simple_arm, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mogi_arm",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", LaunchConfiguration('z'), "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge camera topics
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/gripper_camera/image",
            "/table_camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'gripper_camera.image.compressed.jpeg_quality': 75,
             'table_camera.image.compressed.jpeg_quality': 75,},
        ],
    )

    # Relay node to republish camera_info to image/camera_info
    relay_gripper_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['gripper_camera/camera_info', 'gripper_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Relay node to republish camera_info to image/camera_info
    relay_table_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['table_camera/camera_info', 'table_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            '--param-file',
            robot_controllers,
            ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(viewer1_name_arg)
    launchDescriptionObject.add_action(viewer2_name_arg)
    launchDescriptionObject.add_action(viewer3_name_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(hand_controller_viewer_node)    
    launchDescriptionObject.add_action(gripper_camera_viewer_node)    
    launchDescriptionObject.add_action(table_camera_viewer_node)    
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_gripper_camera_info_node)
    launchDescriptionObject.add_action(relay_table_camera_info_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    launchDescriptionObject.add_action(joint_trajectory_controller_spawner)

    return launchDescriptionObject
