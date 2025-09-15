from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mavros',
            executable = 'mavros_node',
            name = 'mavros',
            output = 'screen',
            parameters = [{
                'fcu_url': '/dev/ttyACM0:57600', # Pixhawk port & baudrate
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1
            }]
        ),

        Node(
            package='drone_manual',
            executable='joy_subscriber',           
            name='joy_node',
            output='screen'
        )
    ])