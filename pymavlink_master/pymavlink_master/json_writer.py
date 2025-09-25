# ros2_writer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32
import json
import os

# Define the path for the output file
FILE_PATH = r"/tmp/drone_data.json"

class JsonWriterNode(Node):
    def __init__(self):
        super().__init__('json_writer_node')
        
        # Create the subscriber
        self.feature_sub = self.create_subscription(String,'drone_data',self.data_callback,10) # QoS profile depth
        self.tel_sub = self.create_subscription(String,'tel_data',self.tel_callback,10) # QoS profile depth
        self.data = [0,0,0,0,0,0]
        self.get_logger().info("JSON writer node started, listening to /drone_position.")

    def data_callback(self, msg):
        for i in msg.data:
            for j in range(4):
                self.data[j+2] = msg.data
        with open(r"/tmp/drone_data.json") as file:
            json.dump(self.data,file)
    
    def tel_callback(self, msg):
        for i in msg.data:
            for j in range(2):
                self.data[j] = msg.data
        with open(r"/tmp/drone_data.json") as file:
            json.dump(self.data,file)
        return
    
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