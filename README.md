# optical_flow
using Arduino Nano, PAA5102 Optical Flow Sensor and MCP25625 CAN shield

# Dependencies
1. Install ros_lib.
2. Install this library: https://github.com/crmkrl/MCP_CAN_lib


# RUN
Streaming CAN Messages 

For ROS, run this command: 

> rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
