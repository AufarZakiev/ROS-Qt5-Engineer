#ifndef ROBOTCONFIGURATION_H
#define ROBOTCONFIGURATION_H


/*
 * This class is responsible for robot's configuration
 * It configures its speed via settingsDialog class
 */
class RobotConfiguration
{

public:
    RobotConfiguration();
    ~RobotConfiguration();
    RobotConfiguration(int platformForwardSpeed=5000, int platformRotateSpeed = 5000, int shouldersSpeed=12000,
                       int neckSpeed=12000,int elbowSpeed=12000, int waistSpeed=8000, bool light = false);

    //these values are the MAX_SPEED of each joint
    int platformForwardSpeed;
    int platformRotateSpeed;
    bool lightValue;
    int shouldersSpeed;
    int neckSpeed;
    int elbowSpeed;
    int waistSpeed;
};

#endif // ROBOTCONFIGURATION_H
