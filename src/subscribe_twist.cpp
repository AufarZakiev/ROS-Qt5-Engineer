// includes of self-wrote libs
#include "robot.h"

// class for subscribing and publishing commands to robot
// contains ros::subscriber and robot objects

SubscribeTwist::SubscribeTwist() {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/cmd_vel_mock", 1, &SubscribeTwist::callbackTwist, this);
    //Robot object to send commands
    robot_ = new Robot();
    robot_->connectToEngineer();
}

void SubscribeTwist::callbackTwist(const geometry_msgs::Twist &twist) {
    robot_->moveD(twist.linear.x);
    robot_->moveR(twist.angular.z);
    return;
}
