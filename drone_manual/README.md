# On Raspi
## Make sure You installed mavros

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras -y
```

## Install the Mavros supporting library
```bash
sudo apt install geographiclib-tools -y
sudo geographiclib-get-geoids egm96-5
```

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

```