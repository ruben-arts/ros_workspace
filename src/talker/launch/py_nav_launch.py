from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='talker',
            executable='talker',
            name='talker',
        ),
        Node(
            package='navigator-py',
            executable='navigator',
            name='navigator',
        ),
    ])
