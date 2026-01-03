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
            "model",
            default_value="/root/hand_controller/handsv2.eim",
            description="Name of Edge Impulse FOMO model."
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
        DeclareLaunchArgument(
            "x_t",
            default_value="0.0",
            description="threshold for activation of twist linear.x control."
        ),
        DeclareLaunchArgument(
            "x_a",
            default_value="0.0",
            description="fixed (offset) value for twist linear.x control."
        ),
        DeclareLaunchArgument(
            "x_b",
            default_value="10.0",
            description="scale (multiplier) value for twist linear.x control."
        ),
        DeclareLaunchArgument(
            "z_t",
            default_value="0.0",
            description="threshold for activation of twist angular.z control."
        ),
        DeclareLaunchArgument(
            "z_a",
            default_value="0.0",
            description="fixed (offset) value for twist angular.z control."
        ),
        DeclareLaunchArgument(
            "z_b",
            default_value="10.0",
            description="scale (multiplier) value for twist angular.z control."
        ),
        DeclareLaunchArgument(
            "dials_single",
            default_value="False",
            description="Use single dials (left) for both linear.x and angular.z."
        ),
        Node(
            package='hand_controller',
            executable='usbcam_publisher_node',
            name="usbcam_publisher",
            remappings=[("image_raw", "usbcam_image")]            
        ),
        Node(
            package='hand_controller',
            executable='hand_controller_ei1dials_twist_node',
            name="hand_controller",
            parameters=[
               {"repo_path":LaunchConfiguration("repo_path")},
               {"model":LaunchConfiguration("model")},
               {"verbose":PythonExpression(['"', LaunchConfiguration('verbose'), '" == "True"'])},
               {"use_imshow":PythonExpression(['"', LaunchConfiguration('use_imshow'), '" == "True"'])},
               {"x_t":LaunchConfiguration("x_t")},
               {"x_a":LaunchConfiguration("x_a")},
               {"x_b":LaunchConfiguration("x_b")},
               {"z_t":LaunchConfiguration("z_t")},
               {"z_a":LaunchConfiguration("z_a")},
               {"z_b":LaunchConfiguration("z_b")},
               {"dials_single":PythonExpression(['"', LaunchConfiguration('dials_single'), '" == "True"'])}
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
