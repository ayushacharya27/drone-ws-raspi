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




