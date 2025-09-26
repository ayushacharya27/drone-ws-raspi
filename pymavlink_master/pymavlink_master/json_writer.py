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
        self.get_logger().info("JSON writer node started, listening to /drone_data & tel_data.")
        self.json_data = {}

    def data_callback(self, msg):
        for i in msg.data:
            for j in range(4):
                self.data[j+2] = msg.data
        self.json_data['persons'] = self.data[2]
        self.json_data['structural_integrity'] = self.data[3]
        self.json_data['voices'] = self.data[5]
        with open(r"/tmp/drone_data.json") as file:
            json.dump(self.json_data,file)
    
    def tel_callback(self, msg):
        self.json_data['lat'] = list(msg.data)[0][0]
        self.json_data['lon'] = list(msg.data)[0][1]
        with open(r"/tmp/drone_data.json") as file:
            json.dump(self.json_data,file)
        
    
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