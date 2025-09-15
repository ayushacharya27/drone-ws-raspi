import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32 # For the Time Being this array we will be sending

# This Node is to Run 

''' Kenny's Model Format
First Input:
   0  1  2  3  .......

0  0  23 45 7  .......

1  23 0  31 12 .......

2  45 31 0  11 .......
.   
.

Second Iteration:
Params Array: {Temperature, Structural_Integrity, Sound_Score, <more planned parameters>}
'''
class ParamsNode(Node):
    def __init__(self):
        super().__init__('rgb_params_node')


        # All Four Parameters
        self.sub_image = self.create_subscription(Image, 'camera/image_raw', self.ParamsCamera, 10)
        self.sub_temp = self.create_subscription(Int32, 'tcam/temp_data', self.TempCallback, 10)
        self.sub_sound = self.create_subscription(AudioData, 'mic/mic_data', self.ParamsSound, 10)

        # Array as a Publisher for GNN Node
        self.final_array = [0, 0, 0, 0]
        self.publisher = self.create_publisher(Int32MultiArray, 'params/param_data', 10)

        self.bridge = CvBridge()
        


    def ParamsCamera(self, msg: Image):
        # Creating Back to Frames
        frame = self.bridge.imgmsg_to_cv2(msg)

        # Prerna Writes Code Here
        # 1. Implment YOLO on frames and Finds Human Density Score {add to final_array[1]}
        # 2. Finds Structural Integrity Score by Corner Detection {add to final_array[2]}
        # 3. ............


        


    def ParamsSound(self, msg: AudioData):
        # Fourier Analysis {add to final_array[3]}
        self.final_array[3] = 32 # Dummy Value




    def TempCallback(self, msg: Int32):
        # Add to final_array[0] only once coz Mother Drone only Maps once
        self.final_array[0] = 98 # Dummy Variable


    # For Publishing Final Array
    def pub_arr(self):
        msg = Int32MultiArray()
        msg.data = self.final_array
        self.publisher.publish(msg)
        



def main(args=None):
    rclpy.init(args=args)
    node = ParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




    
















