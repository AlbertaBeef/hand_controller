from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            default_value="blaze_qairt",
            description="blaze target implementation."
        ),       
        DeclareLaunchArgument(
            "blaze_model1",
            default_value="palm_detection_full.bin",
            description="Name of blaze detection model."
        ),       
        DeclareLaunchArgument(
            "blaze_model2",
            default_value="hand_landmark_full.bin",
            description="Name of blaze landmark model."
        ),
        DeclareLaunchArgument(
            "threshold_detector_minscore",
            default_value="0.6",
            description="Minimum Score threshold for detection model."
        ),
        DeclareLaunchArgument(
            "threshold_detector_nms",
            default_value="0.3",
            description="NMS threshold for detection model."
        ),
        DeclareLaunchArgument(
            "threshold_landmark_confidence",
            default_value="0.5",
            description="Confidence Score threshold for landmark model."
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="True",
            description="Verbose mode."
        ),               
        DeclareLaunchArgument(
            "use_imshow",
            default_value="False",
            description="Enable OpenCV display."
        ),
        DeclareLaunchArgument(
            "use_motor_bridge",
            default_value="True",
            description="Include motor bridge for LeKiwi mobile base robot."
        ),        
        Node(
            package='hand_controller',
            executable='usbcam_publisher_node',
            name="usbcam_publisher",
            remappings=[("image_raw", "usbcam_image")]            
        ),
        Node(
            package='hand_controller',
            executable='hand_controller_qai2asl_twist_node',
            name="hand_controller",
            parameters=[
               {"repo_path":LaunchConfiguration("repo_path")},
               {"blaze_target":LaunchConfiguration("blaze_target")},
               {"blaze_model1":LaunchConfiguration("blaze_model1")},
               {"blaze_model2":LaunchConfiguration("blaze_model2")},
               {"threshold_detector_minscore":LaunchConfiguration("threshold_detector_minscore")},
               {"threshold_detector_nms":LaunchConfiguration("threshold_detector_nms")},
               {"threshold_landmark_confidence":LaunchConfiguration("threshold_landmark_confidence")},
               {"verbose":PythonExpression(['"', LaunchConfiguration('verbose'), '" == "True"'])},
               {"use_imshow":PythonExpression(['"', LaunchConfiguration('use_imshow'), '" == "True"'])}
            ],
            remappings=[
               ("image_raw", "usbcam_image"),
               ("hand_controller/cmd_vel", "cmd_vel")
            ]            
        ),
        Node(
            package='lekiwi_hw_interface',
            executable='lekiwi_motor_bridge',
            name='lekiwi_motor_bridge',
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('use_motor_bridge'), '" == "True"']))            
        )
    ])
