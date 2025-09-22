import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.capture = cv2.VideoCapture(0)  

    def timer_callback(self):
        ret, frame = self.capture.read()
        if ret:
            # Compress frame as JPEG
            _, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = encoded_img.tobytes()

            self.publisher.publish(msg)
            #self.get_logger().info("Published compressed frame")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
