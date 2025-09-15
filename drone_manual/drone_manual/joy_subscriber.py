import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_to_rc_override')
        self.subscription = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming') # For Arming
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode') # For Changing Mode

        self.current_state = State()
        self.last_buttons = []

        self.get_logger().info("Mavros bridge started")

    def state_callback(self, msg: State):
        self.current_state = msg

    def joy_callback(self, msg: Joy):
        # Write According to ArduPilot


        # Joel and Me Have to Discuss

        # Modes Set for ARM/DISARM and Set_Modes

        # To Keep Track of Last Pressed Buttons
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons) # Intial Array [0,0,0] 3 Buttons A, B, Y

        # A button (ARM)
        if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
            arm_state = not self.current_state.armed
            self.get_logger().info(f"{'ARMED' if arm_state else 'DISARMED'}")
            self.arm_func(arm_state)

        # B button (MANUAL)
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            self.get_logger().info("On Manual")
            self.mode_func("MANUAL")

        # Y button (STABILIZE)
        if msg.buttons[3] == 1 and self.last_buttons[3] == 0:
            self.get_logger().info("On Stabilize")
            self.mode_func("STABILIZE")

        self.last_buttons = msg.buttons

    def arm_func(self, arm: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Arm/Disarm Service unavailable")
            return
        # For Getting Verification That Command Succesful or not (Not Required)
        req = CommandBool.Request()
        req.value = arm
        future = self.arm_client.call_async(req)

        def done_cb(fut):
            res = fut.result()
            if res and res.success:
                self.get_logger().info(f"{'ARMED' if arm else 'DISARMED'}")
            else:
                self.get_logger().warning("Arm/Disarm Failed")

        future.add_done_callback(done_cb)

    def mode_func(self, mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Service Unavailable")
            return

        # For Getting Verification That Command Succesful or not (Not Required)
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)

        def done_cb(fut):
            res = fut.result()
            if res and res.mode_sent:
                self.get_logger().info(f"Mode set to {mode}")
            else:
                self.get_logger().warning(f"Womp Womp!! {mode}")

        future.add_done_callback(done_cb)


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
