from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scenario_file = LaunchConfiguration("scenario_file")
    
    return LaunchDescription([
        DeclareLaunchArgument("scenario_file", default_value="",
            description="Path to the fault injection scenario YAML file",
            ),
        Node(
            package="ros2_fault_injection",
            executable="fault_injector_node",
            name="fault_injector_node",
            output="screen",
            parameters=[{"scenario_file": scenario_file},
            ],
        ),  
    ])
