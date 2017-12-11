#ifndef ROBOTSI_H
#define ROBOTSI_H
#include "robot.h"

class RobotSI : public Robot
{
public:
    RobotSI();
    void moveD(double speed);
    void moveR(double speed);

    static int msToMotorSpeedForward(double speed);
    static int msToMotorSpeedRotate(double speed);
    static double motorSpeedToMsForward(int speed);
    static double motorSpeedToMsRotate(int speed);
private:
    constexpr static double forwardSpeedKoeff = 71553.83671;
    constexpr static double rotateSpeedKoeff = 25171.21554;

};

#endif // ROBOTSI_H
