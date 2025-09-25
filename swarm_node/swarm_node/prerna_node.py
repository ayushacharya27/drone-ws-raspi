import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CompressedImage
#from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
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
        self.sub_image = self.create_subscription(CompressedImage, 'camera/image_raw', self.ParamsCamera, 10)
        #self.sub_temp = self.create_subscription(Int32, 'tcam/temp_data', self.TempCallback, 10)
        #self.sub_sound = self.create_subscription(AudioData, 'mic/mic_data', self.ParamsSound, 10)

        # Array as a Publisher for GNN Node
        self.final_array = [0, 0, 0, 0]
        #self.publisher = self.create_publisher(Int32MultiArray, 'params/param_data', 10)

        #self.bridge = CvBridge()
        self.model = YOLO("/home/ayush/drone-ws-code/swarm_node/swarm_node/best.pt")


    def ParamsCamera(self, msg: CompressedImage):
        # Creating Back to Frames
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame, verbose=False)
        detections = results[0].boxes  # YOLOv8 uses .boxes for bounding boxes
        person_indices = [i for i, cls in enumerate(detections.cls) if int(cls) == 0]  # class 0 = person

    # Create a copy of frame to draw only persons
        detect_frame = frame.copy()
        for i in person_indices:
            box = detections.xyxy[i]  # bounding box: [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(detect_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(detect_frame, "Person", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        #annotated_frame = results[0].plot()
        cv2.imshow("YOLO detections", detect_frame)
        cv2.waitKey(1)


        # Prerna Writes Code Here
        # 1. Implment YOLO on frames and Finds Human Density Score {add to final_array[1]}
        # 2. Finds Structural Integrity Score by Corner Detection {add to final_array[2]}
        # 3. ............


        


    '''def ParamsSound(self, msg: AudioData):
        # Fourier Analysis {add to final_array[3]}
        self.final_array[3] = 32 # Dummy Value'''
    




    '''def TempCallback(self, msg: Int32):
        # Add to final_array[0] only once coz Mother Drone only Maps once
        self.final_array[0] = 98 # Dummy Variable'''


    # For Publishing Final Array
    '''def pub_arr(self):
        msg = Int32MultiArray()
        msg.data = self.final_array
        self.publisher.publish(msg)'''
        



def main(args=None):
    rclpy.init(args=args)
    node = ParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




    
















