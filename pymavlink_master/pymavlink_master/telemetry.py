import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from pymavlink import mavutil
import json

path = r"/tmp/drone_data.json"

class PixhawkTelemetry(Node):
    def __init__(self, udp_port='udp:127.0.0.1:14551'):
        super().__init__('pixhawk_telemetry')

        # Publishers
        self.rc_pub = self.create_publisher(Int32MultiArray, 'rc_channels', 10)
        self.gps_pub = self.create_publisher(Float32MultiArray, 'gps', 10)   # [lat, lon, alt]
        self.yaw_pub = self.create_publisher(Float32MultiArray, 'imu_yaw', 10) # [yaw]
        self.acc_pub = self.create_publisher(Float32MultiArray, 'imu_acc', 10) # [accx,accy,accz]

        self.drone_data = self.create_publisher(String, 'drone_data', 10) # complete drone data

        # MAVLink connection
        self.master = mavutil.mavlink_connection(udp_port)
        self.master.wait_heartbeat()
        self.get_logger().info(f"✅ Connected to Pixhawk system {self.master.target_system}")

        # Timer @20Hz
        self.timer = self.create_timer(0.05, self.read_telemetry)

    def read_telemetry(self):
        msg = self.master.recv_match(blocking=False)

        if not msg:
            return

        mtype = msg.get_type()

        # RC Channels
        if mtype == 'RC_CHANNELS':
            rc_values = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw
            ]
            self.rc_pub.publish(Int32MultiArray(data=rc_values))
            self.get_logger().info(f"🎮 RC Channels: {rc_values}")

        # GPS
        elif mtype == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7   # degrees
            lon = msg.lon / 1e7   # degrees
            alt = msg.alt / 1000.0  # meters
            gps_data = [lat, lon, alt]
            self.gps_pub.publish(Float32MultiArray(data=gps_data))
            self.get_logger().info(f"📡 GPS: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}m")

        # IMU Yaw
        elif mtype == 'ATTITUDE':
            yaw = msg.yaw  # radians
            yaw_deg = yaw * 180.0 / 3.14159265359
            self.yaw_pub.publish(Float32MultiArray(data=[yaw_deg]))
            self.get_logger().info(f"🧭 Yaw: {yaw_deg:.2f}°")

        # IMU ACC
        elif mtype == 'SCALED_IMU2':
            accel = Float32MultiArray()
            g = 9.80665  # Standard gravity
            accel_x = msg.xacc * (g / 1000.0)
            accel_y = msg.yacc * (g / 1000.0)
            accel_z = msg.zacc * (g / 1000.0)
            accel.data = [accel_x,accel_y,accel_z]
            self.acc_pub.publish(accel)

        data = [gps_data, accel]

        with open(path,"w") as file:
            json.dump(data, file)
        


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
