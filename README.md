# ESP32 Car

ESP32-CAM + L298N driver for ROS.

![Wiring Diagram](schematic.png)

## Installation

```
pip install websocket-client

sudo apt-get install ros-noetic-video-stream-opencv
```

## Usage


```
roslaunch esp32_ros ackermann.launch

roslaunch esp32_ros camera.launch

roslaunch esp32_ros automove.launch
```
