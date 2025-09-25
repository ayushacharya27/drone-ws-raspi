from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'mavproxy.py',
                '--master=/dev/pixhawk',
                '--baudrate=11520',
                '--out=udp:127.0.0.1:14550',
                '--out=udp:127.0.0.1:14551',
                '--out=udp:127.0.0.1:14552',
                ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'streamlit', 'run', '/home/ayush/drone-ws-code/survey/survey/map_interface.py'
            ],
            output='screen'
        ),


        Node(
            package='pymavlink_master',
            executable='start_link',
            name = 'start_link',
            output = 'screen',
        ),
        Node(
            package='survey',
            executable='survey',
            name = 'survey',
            output = 'screen',
        ),

        
    ])