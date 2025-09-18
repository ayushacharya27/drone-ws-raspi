import math
import geopy
import geopy.distance
from pymavlink import mavutil
from pymavlink.mavutil import mavlink

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

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

# get value
survey_area = [
    (12.8713, 80.2221),
    (12.8719, 80.2223),
    (12.8717, 80.2235),
    (12.8711, 80.2233)
]

def node_at_least_distance():
    for i in len(survey_area):
        distance = geopy.distance.distance(tuple(current_pos), survey_area[i])
        distances_from_current.append((distance,i))
    return(min(distances_from_current)[1])

# returns [longer,shorter] node
def longer_side():
    global first_node
    distance1 = geopy.distance.distance(survey_area[first_node], survey_area[first_node-1])
    distance2 = geopy.distance.distance(survey_area[first_node], survey_area[first_node+1])
    return([max((distance1, int(first_node-1)),(distance2, int(first_node+1)))[1], min((distance1, int(first_node-1)),(distance2, int(first_node+1)))[1]])

def calculate_bearing(start, end):
    #Calculates the bearing between two GPS coordinates.
    lat1_rad = math.radians(survey_area[node_at_least_distance][0])
    lon1_rad = math.radians(survey_area[node_at_least_distance][1])
    lat2_rad = math.radians(survey_area[longer_side][0])
    lon2_rad = math.radians(survey_area[longer_side][1])

    # Formula to find the bearing
    delta_lon = lon2_rad - lon1_rad
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    # Calculate the angle and convert from radians to degrees
    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)

    # Normalize the bearing to be between 0 and 360 degrees
    return (bearing_deg + 360) % 360

def length(point,dir):
    dist = 0
    distance_to_travel = geopy.distance.distance(meters=trigger_spacing)
    while dist <= long_side:
        next_point = distance_to_travel.destination(point=point, bearing=dir)
        waypoints.append((next_point.latitude,next_point.longitude))
        point = next_point
        dist += trigger_spacing
    return breadth()

def breadth():
    global forward, total_dist
    if total_dist <= short_side:
        distance_to_travel = geopy.distance.distance(meters=line_spacing)
        next_point = distance_to_travel.destination(point=waypoints[-1], bearing=forward)
        waypoints.append((next_point.latitude,next_point.longitude))
        total_dist += line_spacing
        return length(waypoints[-1], )
    return 

def generate_waypoints():
    global waypoints
    waypoints = []
    loc = (0,0)
    lat, lon = loc

    return

global first_node, seccond_node, zig, zag, long_side, short_side, waypoints, total_dist

first_node = node_at_least_distance()
second_node = longer_side()[0]
third_node = longer_side()[1]

zig = calculate_bearing(first_node, second_node)
zag = calculate_bearing(second_node,first_node)
forward = calculate_bearing(first_node, third_node)

long_side = geopy.distance.distance(survey_area[first_node], survey_area[second_node])
short_side = geopy.distance.distance(survey_area[first_node], survey_area[third_node])
total_dist = 0

waypoints = []