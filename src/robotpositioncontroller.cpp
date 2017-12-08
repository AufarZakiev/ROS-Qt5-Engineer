#include "../headers/robotpositioncontroller.h"
#include <QObject>
#include <QByteArray>
#include <QStringBuilder>
#include <QString>
#include "../headers/myudpclient.h"
#include "../headers/robotpackets.h"
#include "QDebug"
#include "../headers/robot.h"
#include <cstdlib>

RobotPositionController::RobotPositionController(Robot *r):QObject()
{
    robot = r;
    timer = new QTimer();
    positionInfo = new TelemetryPacket;
    connect(timer, SIGNAL(timeout()), this, SLOT(rotateElbow()));
}

RobotPositionController::~RobotPositionController(){
    delete positionInfo;
}

void RobotPositionController::handleTelemetry(char *data){

    delete positionInfo;
    positionInfo = (TelemetryPacket*) data;
}


void RobotPositionController::rotateWaist(){
    qDebug("ENTERED");

    int delta = executePositionValue(angle);
    if(deltaApproximateEquality(this->startTelemetry, positionInfo->M_DATA[5].POSITION, delta)) {
      timer->stop();
      robot->stopAll();
    }
}

void RobotPositionController::rotateElbow(){
    int delta = executePositionValue(angle);
    if(deltaApproximateEquality(this->startTelemetry, positionInfo->M_DATA[2].POSITION, delta)) {
      timer->stop();
      robot->stopAll();
    }
}

void RobotPositionController:: startTimerTask(int angle){
    //this->startTelemetry = positionInfo->M_DATA[5].POSITION;
    this->startTelemetry = positionInfo->M_DATA[2].POSITION;
    this->angle = angle;
    timer->start(100);
//    if ( angle > 0 )
//        robot->turnWaist(10000);
//    else robot->turnWaist(-10000);
        if ( angle > 0 )
            robot->turnElbowAndNeck(10000);
        else robot->turnElbowAndNeck(-10000);
}

int RobotPositionController::executePositionValue(int angle){
    return ((65535*angle)/360);
}

bool RobotPositionController::deltaApproximateEquality(int first_telemtry, int current_telemetry, int angle_delta){
    if (angle_delta>0){
    if((std::abs(current_telemetry-first_telemtry)+ANGLE_DELTA) > angle_delta) return true;
        return false;
    }else
    {
        if((-std::abs(current_telemetry-first_telemtry)-ANGLE_DELTA) < angle_delta) return true;
            return false;

    }
}
