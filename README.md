# Engineer control node

## Overview

ROS node for "Engineer" robot control through Qt libraries for UDP protocol.

## Features

Node provides "Engineer" robot control using <geometry_msgs::Twist> type topic.

## Usage

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