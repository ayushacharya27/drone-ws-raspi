import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode

class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_to_mavros')

        # Subscriptions
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Publishers
        self.manual_pub = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)

        # Services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.last_buttons = []

        self.get_logger().info("🎮 Mavros Joystick Bridge Started")

    def state_callback(self, msg: State):
        self.current_state = msg

    def joy_callback(self, msg: Joy):
        # --- Axis Mapping for ManualControl ---
        manual = ManualControl()

        # Map axes [-1,1] to Pixhawk expected range [-1000,1000]
        # Adjust axes index based on your joystick
        manual.x = float(msg.axes[0] * 1000.0)  # Roll (Left stick X)
        manual.y = float(msg.axes[1] * 1000.0)  # Pitch (Left stick Y)
        manual.z = float(msg.axes[3] * 1000.0)  # Throttle (Right stick Y)
        manual.r = float(msg.axes[2] * 1000.0)  # Yaw (Right stick X)
        manual.buttons = 0  # Default

        self.manual_pub.publish(manual)
        self.get_logger().debug(f"Published ManualControl: x={manual.x}, y={manual.y}, z={manual.z}, r={manual.r}")

        # --- Button Handling for ARM/DISARM + Modes ---
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)

        # A button (index 0) → ARM/DISARM toggle
        if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
            arm_state = not self.current_state.armed
            self.get_logger().info(f"{'ARMING' if arm_state else 'DISARMING'}")
            self.arm_func(arm_state)

        # B button (index 1) → ACRO
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            self.get_logger().info("Switching to ACRO mode")
            self.mode_func("ACRO")

        # Y button (index 3) → STABILIZE
        if msg.buttons[3] == 1 and self.last_buttons[3] == 0:
            self.get_logger().info("Switching to STABILIZE mode")
            self.mode_func("STABILIZE")

        self.last_buttons = msg.buttons

    def arm_func(self, arm: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("⚠️ Arm/Disarm Service unavailable")
            return

        req = CommandBool.Request()
        req.value = arm
        future = self.arm_client.call_async(req)

        def done_cb(fut):
            res = fut.result()
            if res and res.success:
                self.get_logger().info(f"{'✅ ARMED' if arm else '❌ DISARMED'}")
            else:
                self.get_logger().warning("⚠️ Arm/Disarm Failed")

        future.add_done_callback(done_cb)

    def mode_func(self, mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("⚠️ Mode Service Unavailable")
            return

        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)

        def done_cb(fut):
            res = fut.result()
            if res and res.mode_sent:
                self.get_logger().info(f"✅ Mode set to {mode}")
            else:
                self.get_logger().warning(f"⚠️ Failed to set mode {mode}")

        future.add_done_callback(done_cb)

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
