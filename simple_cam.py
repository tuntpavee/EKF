import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedCam(Node):
    def __init__(self):
        super().__init__('compressed_cam_node')
        
        # Publish CompressedImage (much smaller = faster)
        self.publisher_ = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        self.timer = self.create_timer(0.033, self.timer_callback) # 30 FPS target
        
        # Open Camera Index 8
        self.cap = cv2.VideoCapture(8)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.format = "jpeg"
            
            # Compress to JPEG
            # Quality 0-100 (50 is a good balance of speed/quality)
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            msg.data = np.array(buffer).tobytes()
            
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CompressedCam()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
