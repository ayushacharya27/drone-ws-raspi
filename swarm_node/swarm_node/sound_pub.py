import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sounddevice as sd
import numpy as np

class SoundPublisher(Node):
    def __init__(self):
        super().__init__('sound_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'human_voice', 10)
        self.samplerate = 16000  # 16 kHz
        self.duration = 5.0       # 1 second chunks
        self.timer = self.create_timer(self.duration, self.publish_sound)

    def publish_sound(self):
       
        waveform = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int32')
        sd.wait()
        
        msg = Int32MultiArray()
        msg.data = waveform.flatten().tolist()
        
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published audio chunk of length {len(msg.data)}')

def main(args=None):
    rclpy.init(args=args)
    node = SoundPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
