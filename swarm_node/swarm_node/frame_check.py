import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedCamSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_cam_sub')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',  # your topic
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        # Decode JPEG/PNG bytes to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CompressedCamSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
