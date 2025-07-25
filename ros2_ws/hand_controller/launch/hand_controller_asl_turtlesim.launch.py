from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "repo_path",
            default_value="/root/hand_controller",
            description="Path (absolute) to hand_controller repo."
        ),
        DeclareLaunchArgument(
            "blaze_target",
            default_value="blaze_tflite",
            description="blaze target implementation."
        ),       
        DeclareLaunchArgument(
            "blaze_model1",
            default_value="palm_detection_lite.tflite",
            description="Name of blaze detection model."
        ),       
        DeclareLaunchArgument(
            "blaze_model2",
            default_value="hand_landmark_lite.tflite",
            description="Name of blaze landmark model."
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name="usbcam_publisher",
            remappings=[("image_raw", "usbcam_image")]                        
        ),
        Node(
            package='hand_controller',
            executable='hand_controller_asl_twist_node',
            name="hand_controller",
            parameters=[
               {"repo_path":LaunchConfiguration("repo_path")},
               {"blaze_target":LaunchConfiguration("blaze_target")},
               {"blaze_model1":LaunchConfiguration("blaze_model1")},
               {"blaze_model2":LaunchConfiguration("blaze_model2")},
               {"verbose":True},
               {"use_imshow":True}
            ],
            remappings=[
               ("image_raw", "usbcam_image"),
               ("hand_controller/cmd_vel", "turtle1/cmd_vel")
            ]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name="my_turtlesim",
        )
    ])
