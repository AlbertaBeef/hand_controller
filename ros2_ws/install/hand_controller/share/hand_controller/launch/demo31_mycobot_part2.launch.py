from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "viewer1_name",
            default_value="Hand Controller",
            description="Name of Image Viewer for hand_controller/annotations."
        ),
        Node(
            package='hand_controller',
            executable='usbcam_subscriber_node',
            name="hand_controller_annotations",
            parameters=[
               {"viewer_name":LaunchConfiguration("viewer1_name")}
            ],
            remappings=[
            	("image_raw", "hand_controller/image_annotated")
            ]
        ), 
        Node(
            package='asl_moveit_demos',
            executable='pose_controlled_moveit',
            name="my_pose_controller",
            remappings=[("target_pose", "target_pose")]
        )
    ])
