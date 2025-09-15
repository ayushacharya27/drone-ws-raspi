import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import Joy
from mavros_msgs.msg import ManualControl

# Throttle(Up/Down) : Left JoyStick (Up and Down)
# Left/Right : Nil
# Forward/Backward : Right JoyStick (Up and Down)
# Arm/Disarm : Button A



class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')

        self.subscription = self.create_subscription(Joy, "/joy", self.mavlink_callback, 10)
        self.publisher = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.armed = 0
    def mavlink_callback(self , msg: Joy):
        manual = ManualControl()
        manual.z = float(msg.axes[1] * 1000)  # Throttle
        manual.y = float(0)  # Left/Right Was Fukcing Up so Removed it
        manual.x = float(msg.axes[4] * 1000)  # Forward/Back

        manual.r = float(0) # Yaw

        self.publisher.publish(manual)

        if msg.buttons[0] == 1:
            if self.armed == 0 :
                self.arm_disarm(True)
                self.armed = 1
            else:
                self.arm_disarm(False)
                self.armed = 0

        if msg.buttons[1] == 1: # Button B
            self.set_mode("MANUAL")
        
        if msg.buttons[3] == 1: # Button Y
            self.set_mode("STABILIZED")


    def arm_disarm(self, arm: bool):

        # Just a Precautionary Exercise
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Arm service not available')
            return

        req = CommandBool.Request()
        req.value = arm
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Armed Brahh" if arm else "Disarmed Brahh")
        else:
            self.get_logger().error("Womp Womp")

    def set_mode(self, mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('SetMode service not available')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Sentry Done is on {mode} mode")
        else:
            self.get_logger().error(f"Failed to set mode {mode}")


 

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode() # Creating a Object for Joy Node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
