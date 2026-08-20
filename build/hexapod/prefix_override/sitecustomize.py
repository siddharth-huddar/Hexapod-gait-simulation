import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/siddharth/ros2_ws/src/hexapod/install/hexapod'
