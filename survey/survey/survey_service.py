from interfaces.srv import Area

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json

class Survey_Service(Node):
    def __init__(self):
        super().__init__('service')
        self.area_server = self.create_service(Area, 'area', self.survey_define)
        self.survey_area = []

    def survey_define(self, response):
        area = Float32MultiArray()
        path = r"/mnt/Storage/Hackathons/SIH/drone-ws-raspi/survey/survey/details.json"
        with open(path, "r") as file:
            self.survey_area = json.load(file)
        area.data = self.survey_area
        response.area = area
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Survey_Service()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()