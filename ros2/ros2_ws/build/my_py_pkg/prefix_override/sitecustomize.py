import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/job/repos/SLAM/ros2/ros2_ws/install/my_py_pkg'
