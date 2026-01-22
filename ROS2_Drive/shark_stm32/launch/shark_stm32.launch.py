from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='shark_stm32',
            executable='stm32_serial',
            name='stm32_serial',
            
        ),
    ])