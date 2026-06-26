from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_1',
            executable='quaternion',
            #name='',
            parameters=[{'use_sim_time': True}], # Enables simulation time
            output='screen'
        )
    ])