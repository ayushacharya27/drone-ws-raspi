# Drone Manual Setup
We used MAVRos here to communicate with the Drone , by setting up our local computer and the edge compute on the same network through a VPN (**tailscale**).

## Steps to Setup Tailscale

### On Edge Compute/Main PC

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```
#### To Login/Start the VPN
Login with the same email ID which we used to login in the host computer
```bash
sudo tailscale up
```
#### Turn Off the VPN
```bash
sudo tailscale down
```

#### Adding Network config
Now in the bash.rc file in both Edge/Ground Station PC, add these and source the bash rc immediately

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Now Source it
```bash
source ~/.bashrc
```


## Manual Control

### Run Joy Node in ROS2 PC
```bash
ros2 run joy joy_node
```
