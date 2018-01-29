// includes of self-wrote libs
#include "subscribe_twist.h"

SubscribeTwist::SubscribeTwist() {
    //Topic you want to subscribe
    ROS_INFO("Constructor started!");
    sub_ = n_.subscribe("/cmd_vel", 1, &SubscribeTwist::callbackTwist, this);
    ROS_INFO("Subscribed!");
    //Robot object to send commands
    robot_ = new RobotSI();
    robot_->connectToEngineer();
    //ROS_INFO("Connecting? %d", robot_->isConnecting);
}

void SubscribeTwist::callbackTwist(const geometry_msgs::Twist &twist) {
    float min_linear = 0.2;
    if (fabs(twist.linear.x) < min_linear && twist.linear.x != 0.0) {
        if (twist.linear.x > 0.0) {
            ROS_INFO("Lin: %f.", min_linear);
            robot_->moveD(min_linear);
        } else {
            ROS_INFO("Lin: %f.", -min_linear);
            robot_->moveD(-min_linear);
        }
    } else {
        robot_->moveD(twist.linear.x);
        ROS_INFO("Lin: %f.", twist.linear.x);
    }
    robot_->moveR(-twist.angular.z);
    ROS_INFO("Ang: %f", -twist.angular.z);
}

SubscribeTwist::~SubscribeTwist() {
    robot_->moveD(0.0);
    robot_->moveR(0.0);
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

