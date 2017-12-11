#ifndef SUBSCRIBE_TWIST_H
#define SUBSCRIBE_TWIST_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "robotsi.h"

// class for subscribing and publishing commands to robot
// contains ros::subscriber and robot objects

class SubscribeTwist {
public:
    SubscribeTwist();

    void callbackTwist(const geometry_msgs::Twist &twist);

    virtual ~SubscribeTwist();

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    RobotSI *robot_;
};

#endif // SUBSCRIBE_TWIST_H
