#include "../headers/robotconfiguration.h"

RobotConfiguration::RobotConfiguration()
{
}
RobotConfiguration::RobotConfiguration(int platformForwardSpeed, int platformRotateSpeed, int shouldersSpeed, int neckSpeed, int elbowSpeed, int waistSpeed, bool light){
    this->platformForwardSpeed = platformForwardSpeed;
    this->platformRotateSpeed = platformRotateSpeed;
    this->shouldersSpeed = shouldersSpeed;
    this->neckSpeed = neckSpeed;
    this->elbowSpeed = elbowSpeed;
    this->waistSpeed = waistSpeed;
    this->lightValue = light;
}


RobotConfiguration::~RobotConfiguration(){}
