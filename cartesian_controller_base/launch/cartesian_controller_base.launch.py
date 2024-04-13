from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a Node instance with the full path to the executable
    node = Node(
        package="cartesian_controller_base",
        executable="cartesian_controller_base",
        name="cartesian_controller",
        output="screen"
    )

    return LaunchDescription([node])
