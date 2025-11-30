import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

class DynamicTF(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        
        # Future timestamp to ensure sync
        future_time = self.get_clock().now() + Duration(seconds=0.2)
        t.header.stamp = future_time.to_msg()
        t.header.frame_id = 'base_link'
        
        # --- FIXED NAME HERE ---
        t.child_frame_id = 'lidar_link' 
        # -----------------------
        
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(DynamicTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
