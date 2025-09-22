## Launch Files Actions

### Imp Points After creating a launch file
#### Step 1: Add the launch file in 
```bash
ros2_ws/<package_name>/launch
```
#### Step 2: Add it to setup.py before building it
```bash
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/master_launch.py']), <------ This Part
    ],
```
#### Step 3: Build it and run
```bash
ros2 launch <package name> <launch file name>.py
```

### For pymavlink_master (can add the survey file launch afterwards)
```bash
ros2 launch pymavlink_master master_launch.py
```

### For Swarm Node (Have to Check the Realsense if Copy paste works or not in Swarm Node )
```bash
ros2 launch swarm_node hover_launch.py
```

## UDEV Rules
1. /dev/pixhawk
2. /dev/webcam (Sony One, coz realsense dont need to have a specific number in code)

## Making New Udev Rules
How to Find Vendor and device ID
```bash
lsusb
                         |     |
                         |     |
                        Ven  devid
Bus 001 Device 004: ID  8086:0b07 Intel Corp. Intel RealSense D435i
```
```bash
sudo nano /etc/udev/rules.d/99-<rule for whom>.rules
```

Paste this (Modify it)
```bash
KERNEL=="video*", ATTRS{idVendor}=="<device_vendor_id>", ATTRS{idProduct}=="<device_id>", SYMLINK+="<name_you_want>"
```

Apply it
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```



## Run Realsense Node
```bash
cd ros2_ws
. install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
Now You can see the topics published


