import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pymavlink import mavutil


class PymavLinkMaster(Node):
    def __init__(self, port = "udp:127.0.0.1:14550", mode = "ACRO"): # First Mode is ACROBATICS
        super().__init__('master')
        self.mode = mode
        self.arm_state = False

        self.master = mavutil.mavlink_connection(port, baud=11520)
        self.master.wait_heartbeat()
        self.get_logger().info(f"Connected to Pixhawk {self.master.target_system}")

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.channel_ary = [1000]*8 # Commands to Pixhawk for Movement
        '''
        1. Pitch
        2. Roll
        3. Throttle
        4. Yaw
        5. Forward
        6. Lateral
        7. Servo1 (Not Required)
        8. Servo2 (Not Required)
        '''

        self.prev_buttons = []

    def joy_callback(self, msg:Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0]*len(msg.buttons)

        '''
        A -> Arm (Button 0)
        B -> Disarm (Button 1)
        Y -> Stabilize (Button 4)
        '''

        # Arming and Disarming
        if msg.buttons[0] and not self.prev_buttons[0]:
            self.arm()
        if msg.buttons[1] and not self.prev_buttons[1]:
            self.disarm()
        
        # Sending Throttle and Forward Command just for now
        self.channel_ary[2] = (msg.axes[1] + 1)*500 + 500 # Throttle
        #self.channel_ary[2] = 1600 # Dummy Throttle
        self.channel_ary[4] = (msg.axes[3] + 1)*500 + 500 # Forward

        self.actuate()


    def arm(self):
        if not self.arm_state:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )

            self.arm_state = True
            self.get_logger().info("Armed Meow Meow")

    def disarm(self):
        if self.arm_state:
            self.channel_ary = [1000]*8
            self.actuate()
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )

            self.arm_state = False
            self.get_logger().info("DisArmed Bhow Bhow")

    def set_rc_channel_pwm(self, id, pwm):
        if 1 <= id <= 8: # For All Channels i.e. from one to eight
            rc_values = [65535]*8 # I Dont know but, this is the way to override the rc_channels 
            rc_values[id-1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_values
            )

    def actuate(self):
        self.set_rc_channel_pwm(1, int(self.channel_ary[0]))
        self.set_rc_channel_pwm(2, int(self.channel_ary[1]))
        self.set_rc_channel_pwm(3, int(self.channel_ary[2]))
        self.set_rc_channel_pwm(4, int(self.channel_ary[3]))
        self.set_rc_channel_pwm(5, int(self.channel_ary[4]))
        self.set_rc_channel_pwm(6, int(self.channel_ary[5]))
        self.set_rc_channel_pwm(7, int(self.channel_ary[6]))
        self.set_rc_channel_pwm(8, int(self.channel_ary[7]))


    # Security Reasons
    def destroy_node(self):
        self.channel_ary = [1500]*8
        self.actuate()

        self.disarm()
        self.get_logger().info("Shutting Down Bye Bye")
        super().destroy_node()





def main(args=None):
    rclpy.init(args=args)
    node = PymavLinkMaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()




        









        






