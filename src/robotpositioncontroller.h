#ifndef ROBOTPOSITIONCONTROLLER_H
#define ROBOTPOSITIONCONTROLLER_H
#include <QByteArray>
#include <QObject>
#include <QTimer>
#include "robotpackets.h"
#include "robot.h"

//1 degree of angle in Servosila's joints
#define ANGLE_DELTA 182

/**
 * @brief The RobotPositionController class
 * This class is used for robot manipulating using position values, like degrees of joints
 */
class RobotPositionController : public QObject
{
    Q_OBJECT

int executePositionValue(int angle);
public: Robot *robot;


public:
    //info about current joints position
    TelemetryPacket *positionInfo;

    void startTimerTask(int angle);
    RobotPositionController(Robot *robot);
    ~RobotPositionController();
public Q_SLOTS:
    void handleTelemetry(char *data);
    void rotateWaist();
    void rotateElbow();
private:
    int angle;
    int startTelemetry;
    QTimer *timer;
    bool deltaApproximateEquality(int first_telemtry, int current_telemetry, int angle_delta);

};

#endif // ROBOTPOSITIONCONTROLLER_H
