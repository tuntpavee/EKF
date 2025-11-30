import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class SimpleLidarOdom(Node):
    def __init__(self):
        super().__init__('simple_lidar_odom')

        # 1. Subscriber (LiDAR)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # 2. Publishers (Odom + TF)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State Variables
        self.prev_centroid = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.get_logger().info("Simple LiDAR Odometry Node Started")

    def scan_callback(self, msg):
        # A. Data Pre-processing
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # Handle size mismatch
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        # Filter invalid points
        valid_mask = (ranges < msg.range_max) & (ranges > 0.1) & (np.isfinite(ranges))
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        if len(ranges) < 10:
            return

        # B. Convert to Cartesian
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)

        # C. Centroid Tracking Algorithm
        current_centroid = np.array([np.mean(x_points), np.mean(y_points)])

        if self.prev_centroid is not None:
            delta = current_centroid - self.prev_centroid
            # Inverted logic: Room moves left -> Robot moves right
            dx = delta[0]
            dy = -delta[1]

            # Update Robot Position
            self.robot_x += dx * math.cos(self.robot_theta) - dy * math.sin(self.robot_theta)
            self.robot_y += dx * math.sin(self.robot_theta) + dy * math.cos(self.robot_theta)

        self.prev_centroid = current_centroid

        # D. Publish Results
        current_time = self.get_clock().now()
        self.publish_odometry(current_time)
        self.publish_tf(current_time, msg.header.frame_id)

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.robot_theta)

        self.odom_publisher.publish(odom)

    def publish_tf(self, current_time, laser_frame_id):
        # 1. odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.robot_theta)
        self.tf_broadcaster.sendTransform(t)
        
        # 2. base_link -> laser (Static Glue)
        t2 = TransformStamped()
        t2.header.stamp = current_time.to_msg()
        t2.header.frame_id = "base_link"
        t2.child_frame_id = laser_frame_id # Auto-detects 'laser' or 'lidar_link'
        t2.transform.rotation.w = 1.0 
        self.tf_broadcaster.sendTransform(t2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q = Quaternion()
        q.x, q.y, q.z, q.w = qx, qy, qz, qw
        return q

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLidarOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
