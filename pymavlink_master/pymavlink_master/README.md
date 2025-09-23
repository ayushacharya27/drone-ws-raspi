## To Start MavProxy
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

```

## Make These Parameters to the values mentioned (For Testing)
```bash
MOT_PWM_MIN = 975
DISARM_DELAY = 127
FS_THR_ENABLE = 0
ARMING_CHECK = 0
```

## Running Nodes
```bash
ros2 run pymavlink_mastetr start_link

ros2 run pymavlink_master telemetry_node
```
