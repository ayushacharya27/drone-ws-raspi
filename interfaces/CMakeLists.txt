cmake_minimum_required(VERSION 3.8)

# Declare the project name
project(survey)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Area.srv"
 )

 # Finalize the package
ament_package()