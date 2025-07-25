from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='museum_guide_pkg',
            executable='museum_guide_node',
            name='museum_guide_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='museum_introducer_pkg',
            executable='qr_code_follower',
            name='qr_code_follower_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='museum_introducer_pkg',
            executable='recommand_path_drive',
            name='recommand_path_drive_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
