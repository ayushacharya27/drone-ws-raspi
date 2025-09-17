import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/Storage/Hackathons/SIH/drone-ws-raspi/install/pymavlink_master'
