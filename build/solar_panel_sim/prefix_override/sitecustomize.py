import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/isrraelfcaq/ros2_ws/install/solar_panel_sim'
