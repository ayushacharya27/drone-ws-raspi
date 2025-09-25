#from interfaces.srv import Area
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
import json
import os

class Survey(Node):
    def __init__(self):
        super().__init__('area_pub')
        self.area_pub = self.create_publisher(Float32MultiArray, 'area_pub', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.survey_area = []

    def joy_callback(self, msg:Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0]*len(msg.buttons)
        # Set AUTO Mode and Survey
        if msg.buttons[7] and not self.prev_buttons[7]:
            self.survey_define()

    def survey_define(self):
        area = Float32MultiArray()
        path = r"/mnt/Storage/Hackathons/SIH/drone-ws-raspi/survey/survey/details.json"
        with open(path, "r") as file:
            self.survey_area = json.load(file)
        area.data = self.survey_area
        self.area_pub.publish(area)

    # Security Reasons
    def destroy_node(self):
        self.get_logger().info("Shutting Down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Survey()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()