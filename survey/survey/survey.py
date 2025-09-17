# 1. Define the survey area (bounding box polygon)
# A list of (latitude, longitude) tuples
survey_area = [
    (12.8713, 80.2221),
    (12.8719, 80.2223),
    (12.8717, 80.2235),
    (12.8711, 80.2233)
]

# 2. Define flight and camera parameters
flight_altitude_m = 50.0  # meters
sidelap_percent = 75.0    # 70%
frontlap_percent = 80.0   # 80%
gimbal_pitch_deg = -90    # Point camera straight down

# Example: Sony a6000 sensor
sensor_width_mm = 3.58
sensor_height_mm = 2.02
focal_length_mm = 3.7

footprint_width = (sensor_width_mm / focal_length_mm) * flight_altitude_m
footprint_height = (sensor_height_mm / focal_length_mm) * flight_altitude_m

line_spacing = footprint_width * (1 - (sidelap_percent / 100.0))
trigger_spacing = footprint_height * (1 - (frontlap_percent / 100.0))