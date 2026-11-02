import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configuration from myrobot_moveit_config package
    moveit_config = MoveItConfigsBuilder(
        "myrobot_description", 
        package_name="myrobot_moveit_config"
    ).to_moveit_configs()
    
    # Create the Python node and pass the MoveIt configuration dictionary
    draw_cube_node = Node(
        package="myrobot_motion_demo",
        executable="draw_cube",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )
    
    return LaunchDescription([
        draw_cube_node
    ])
