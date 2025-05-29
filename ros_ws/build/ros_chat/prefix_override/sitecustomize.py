import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/ye/E/milk_v/shared/ros_ws/install/ros_chat'
