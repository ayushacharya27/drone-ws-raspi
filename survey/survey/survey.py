import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pymavlink import mavutil
from pymavlink.mavutil import mavlink
import math
import geopy
import geopy.distance
import json

# Define flight and camera parameters
flight_altitude_m = 50.0  # in meters
sidelap_percent = 75.0  
frontlap_percent = 80.0 
gimbal_pitch_deg = -90    # Point camera straight down

# Sony Web Camera CMU-BR200 (estimate)
sensor_width_mm = 3.58
sensor_height_mm = 2.02
focal_length_mm = 3.7

# actual distace on ground
footprint_width = (sensor_width_mm / focal_length_mm) * flight_altitude_m 
footprint_height = (sensor_height_mm / focal_length_mm) * flight_altitude_m

line_spacing = footprint_width * (1 - (sidelap_percent / 100.0)) # how far apart your flight lines should be
trigger_spacing = footprint_height * (1 - (frontlap_percent / 100.0)) # how often to take a picture along each line (in meter)


current_pos = [12.8713, 80.2221] # get value
distances_from_current = []

class Survey(Node):
    def __init__(self, port = "udp:127.0.0.1:14552"):
        super().__init__('survey')

        self.master = mavutil.mavlink_connection(port, baud=11520)
        self.master.wait_heartbeat()
        self.get_logger().info(f"Connected to Pixhawk {self.master.target_system}")
        
        self.prev_buttons = []
        self.survey_area = []

        self.first_point = self.node_at_least_distance()
        self.second_point = self.longer_side()[0]
        self.third_point = self.longer_side()[1]

        self.zig = self.calculate_bearing(self.survey_area[self.first_point], self.survey_area[self.second_point])
        self.zag = self.calculate_bearing(self.survey_area[self.second_point], self.survey_area[self.first_point])
        self.forward = self.calculate_bearing(self.survey_area[self.first_point], self.survey_area[self.third_point])
        self.bearing = [self.forward, self.zig, self.zag]

        self.long_side = geopy.distance.distance(self.survey_area[self.first_point], self.survey_area[self.second_point])
        self.short_side = geopy.distance.distance(self.survey_area[self.first_point], self.survey_area[self.third_point])
        self.total_dist = 0

        self.waypoints = []

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg:Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0]*len(msg.buttons)

        '''
        A -> Arm (Button 0)
        B -> Disarm (Button 1)
        Y -> Stabilize (Button 4)
        X -> AUTO (Button 2)
        '''

        # Set AUTO Mode and Survey
        if msg.buttons[2] and not self.prev_buttons[2]:
            self.auto_mode()
            self.survey_define()
            self.generate_waypoints()
            self.mission()
            self.upload_mission()
        
        self.prev_buttons = list(msg.buttons).copy()


    def auto_mode(self):
        mode_id = self.master.mode_mapping()["AUTO"]
        self.master.set_mode(mode_id)
        self.get_logger().info("In AUTO Mode")

    def survey_define(self):
        with open("details.json", "w") as file:
            self.survey_area = json.loads(file)

    def node_at_least_distance(self):
        for i in range(len(self.survey_area)):
            distance = geopy.distance.distance(tuple(current_pos), self.survey_area[i])
            distances_from_current.append((distance,i))
        return(min(distances_from_current)[1])

    # returns [longer,shorter] node
    def longer_side(self):
        distance1 = geopy.distance.distance(self.survey_area[self.first_point], self.survey_area[self.first_point-1])
        distance2 = geopy.distance.distance(self.survey_area[self.first_point], self.survey_area[self.first_point+1])
        return([max((distance1, int(self.first_point-1)),(distance2, int(self.first_point+1)))[1],
                 min((distance1, int(self.first_point-1)),(distance2, int(self.first_point+1)))[1]])

    def calculate_bearing(self,start, end):
        #Calculates the bearing between two GPS coordinates.
        lat1_rad = math.radians(start[0])
        lon1_rad = math.radians(start[1])
        lat2_rad = math.radians(end[0])
        lon2_rad = math.radians(end[1])

        # Formula to find the bearing
        delta_lon = lon2_rad - lon1_rad
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

        # Calculate the angle and convert from radians to degrees
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)

        # Normalize the bearing to be between 0 and 360 degrees
        return (bearing_deg + 360) % 360

    def length(self,dir):
        distance_to_travel = geopy.distance.distance(meters=self.long_side)
        next_point = distance_to_travel.destination(point=self.waypoints[-1], bearing=dir)
        self.waypoints.append((next_point.latitude,next_point.longitude))
        return self.breadth(dir)

    def breadth(self,dir):
        distance_to_travel = geopy.distance.distance(meters=line_spacing)
        if self.total_dist <= self.short_side:
            next_point = distance_to_travel.destination(point=self.waypoints[-1], bearing=self.bearing[0])
            self.waypoints.append((next_point.latitude,next_point.longitude))
            self.total_dist += line_spacing
            return self.length(dir*(-1))
        return 

    def generate_waypoints(self):
        self.waypoints = [self.survey_area[self.first_point]]
        self.length(1)
        return

    def upload_mission(self,mission_items):
        """Uploads a mission to the vehicle."""
        print("Uploading mission...")
        self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        
        # Wait for the clear all command to be acknowledged
        self.master.recv_match(type='MISSION_ACK', blocking=True)

        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(mission_items))

        for i in range(len(mission_items)):
            msg = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=3)
            if msg:
                if msg.seq == i:
                    self.master.mav.send(mission_items[i])
                    print(f"Sent waypoint {i}")
            else:
                print("Error: No mission request received.")
                return False

        final_ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if final_ack and final_ack.type == mavlink.MAV_MISSION_ACCEPTED:
            print("Mission upload successful!")
            return True
        else:
            print(f"Mission upload failed with error type: {final_ack.type}")
            return False

    def mission(self):
        mission_items = []
        seq = 0
        alt = 3 # altitude in meters

        # Add a dummy waypoint for the home position (required by some GCS/firmware)
        mission_items.append(mavlink.MAVLink_mission_item_int_message(
            self.master.target_system, self.master.target_component, seq, mavlink.MAV_FRAME_GLOBAL,
            mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, 0, 0, 0))
        seq += 1

        # takeoff command
        mission_items.append(mavlink.MAVLink_mission_item_int_message(
            self.master.target_system, self.master.target_component, seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, 0, 0, alt))
        seq += 1

        # Add the waypoints
        for lat, lon in self.waypoints:
            mission_items.append(mavlink.MAVLink_mission_item_int_message(
                self.master.target_system, self.master.target_component, seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 15, 0, 0, int(lat * 1e7), int(lon * 1e7), alt))
            seq += 1

        # return to launch
        mission_items.append(mavlink.MAVLink_mission_item_int_message(
            self.master.target_system, self.master.target_component, seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
        seq += 1

        # upload mission
        if self.upload_mission(mission_items):
            print("Mission is ready on the vehicle.")

    # Security Reasons
    def destroy_node(self):
        self.get_logger().info("Shutting Down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Survey()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
