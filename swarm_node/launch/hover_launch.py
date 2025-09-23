from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_node',
            executable='rgb_node',
            name = 'rgb_node',
            output = 'screen',
        ),
        Node(
            package='swarm_node',
            executable='param_node',
            name = 'param_node',
            output = 'screen',
        ),

        
    ])