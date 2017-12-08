#include "ros/ros.h"
#include "subscribe_twist.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_control_node");
  //Create an object of class SubscribeGMap that will take care of everything
  SubscribeTwist twistManager;

  ros::spin();
}

