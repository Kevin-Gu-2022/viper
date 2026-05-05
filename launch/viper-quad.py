from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='viper',
            executable='viper_node',
            name='viper',
            namespace='viper',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'can_iface': 'vcan0'},
                {'can_node_id': 100},
            ]
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'echo', 'imu'],
        #     output='screen',
        #     additional_env={'period': '0.1', 'msg': '[100, 100, 100, 100]', 'topic': '113:zubax.primitive.real16.Vector4'}
        # ),

    ])
