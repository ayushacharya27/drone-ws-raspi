# ros2_writer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

# Define the path for the output file
FILE_PATH = r"/tmp/drone_data.json"

class JsonWriterNode(Node):
    def __init__(self):
        super().__init__('json_writer_node')
        
        # Create the subscriber
        self.subscription = self.create_subscription(String,'/drone_data',self.data_callback,10) # QoS profile depth
        
        self.get_logger().info("JSON writer node started, listening to /drone_position.")

    def data_callback(self, msg):
        """Callback to write the received ROS message to a JSON file."""
        
        # Get the current time from the ROS 2 clock
        now = self.get_clock().now()
        timestamp = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9

        # Prepare the data in a dictionary format
        data_to_write = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "timestamp": timestamp
        }
        
        # Safe File Write Pattern (write to temp file, then rename)
        temp_file_path = FILE_PATH + ".tmp"
        with open(temp_file_path, 'w') as f:
            json.dump(data_to_write, f)
        os.rename(temp_file_path, FILE_PATH)
        
        self.get_logger().info(f"Wrote data to {FILE_PATH}")

def main(args=None):
    rclpy.init(args=args)
    json_writer_node = JsonWriterNode()
    try:
        rclpy.spin(json_writer_node)
    except KeyboardInterrupt:
        pass
    finally:
        json_writer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()