from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prm',
            executable='flag_servo',
            name='flag_servo',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/flag_servo_enable', '/flag_servo_enable'),
                ('/scan', '/scan'),
                ('/robot_cam', '/robot_cam'),
                ('/flag_servo_arrived', '/flag_servo_arrived')
            ]
        )
    ])
