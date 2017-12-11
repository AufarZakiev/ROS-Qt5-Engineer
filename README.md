# Engineer control node

## Overview

ROS node for "Engineer" robot control through Qt libraries for UDP protocol.

## Features

Node provides "Engineer" robot control using <geometry_msgs::Twist> type topic.

## Usage

First of all, change following line in `CMakeLists.txt`:
```
list(APPEND CMAKE_PREFIX_PATH /home/aufarz/Qt/5.9.3/gcc_64/lib/cmake)
```
with proper path to your Qt 5 folder.

Then make it:
```sh
catkin_make
```

Launch ROS Master:
```sh
roscore
```

Launch the only node from this repo:
```sh
rosrun ros_qt_controller twist_control_node
```

Use `rostopic echo` command to publish messages into input topic:
```sh
rostopic pub /cmd_vel_mock geometry_msgs/Twist '[0.3, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```
Sample command will make robot move forward by the speed of 0.3 m/s.