from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. LiDAR Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_frame'}],
            output='screen'
        ),
        # 2. Static Transform (Laser is at 0,0,0 of robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),
        # 3. RF2O Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'freq': 10.0
            }],
            output='screen'
        )
    ])
