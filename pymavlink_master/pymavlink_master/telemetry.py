import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil

class PixhawkTelemetry(Node):
    def __init__(self, udp_port='udp:127.0.0.1:14551'):
        super().__init__('pixhawk_telemetry')
        self.publisher = self.create_publisher(Int32MultiArray, 'rc_channels', 10)
        self.master = mavutil.mavlink_connection(udp_port)
        self.master.wait_heartbeat()
        self.get_logger().info(f"Connected to Pixhawk system {self.master.target_system}")

        # Timer to check for messages periodically
        self.timer = self.create_timer(0.05, self.read_channels)  # 20 Hz

    def read_channels(self):
        msg = self.master.recv_match(type='RC_CHANNELS', blocking=False)
        if msg:
            rc_values = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw
            ]
            array_msg = Int32MultiArray(data=rc_values)
            self.publisher.publish(array_msg)
            self.get_logger().info(f"RC Channels: {rc_values}")


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
