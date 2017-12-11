// includes of self-wrote libs
#include "subscribe_twist.h"

SubscribeTwist::SubscribeTwist() {
    //Topic you want to subscribe
    ROS_INFO("Constructor started!");
    sub_ = n_.subscribe("/cmd_vel_mock", 1, &SubscribeTwist::callbackTwist, this);
    ROS_INFO("Subscribed!");
    //Robot object to send commands
    robot_ = new RobotSI();
    robot_->connectToEngineer();
    ROS_INFO("Connecting? %d", robot_->isConnecting);
}

void SubscribeTwist::callbackTwist(const geometry_msgs::Twist &twist) {
    ROS_INFO("In callback: connected? %d", robot_->isConnected);
    ROS_INFO("In callback: connecting? %d", robot_->isConnecting);
    robot_->moveD(twist.linear.x);
    robot_->moveR(twist.angular.z);
    ROS_INFO("Linear x: %f", twist.linear.x);
    ROS_INFO("Angular z: %f", twist.angular.z);
}

SubscribeTwist::~SubscribeTwist() {
    robot_->disconnectFromEngineer();
    delete (robot_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist_control_node");
    //Create an object of class SubscribeTwist that will take care of everything
    SubscribeTwist subscribeManager;
    ROS_INFO("Spinning!");
    ros::spin();
    return 0;
}

