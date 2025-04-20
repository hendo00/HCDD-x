from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
        ),
        Node(
            package='odrive_test',
            executable='odrive_control',
            name='odrive_control'
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['-n','-l', '/home/alsaibie/kuacel_ws/src/odrive_test/plotjug.xml']
        )

    ])
