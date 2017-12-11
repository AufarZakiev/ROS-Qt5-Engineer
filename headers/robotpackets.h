#ifndef ROBOTPACKETS_H
#define ROBOTPACKETS_H

//necessary pragma which removes ZERO bytes
//See Servosila documentation for more details
#pragma pack(push,1)
struct PacketConsts
{
    static const int TELEMETRY_PACKET_SIZE = 275;
    static const int TELEMETRY_PACKET_ID = 2;
    static const int REMOTE_CONTROL_PACKET_SIZE = 57;
    static const int REMOTE_CONTROL_PACKET_ID = 1;
    static const int VIDEO_FRAME_PACKET_ID = 4;
};
struct RemoteControlPacket{
    u_int8_t FRAME_TYPE_ID=1;
    int16_t AXIS[16];
    u_int8_t BUTTON[16];
    double TELEMETRY;
};
struct MOTOR_DATA{
    u_int8_t DEVICE_ID;
    u_int8_t DEVICE_STATE;
    u_int8_t OPERATION_MODE;
    u_int32_t POSITION;
    int16_t SPEED;
    int16_t AMPS;
    int16_t STATUS_BITS;
    u_int32_t POSITION_COMMAND;
    int16_t SPEED_COMMAND;
    int16_t AMPS_COMMAND;
    u_int8_t FAULT_DETECTED;
    u_int16_t FAULTS_COUNTER;

};
struct TelemetryPacket{
    u_int8_t FRAME_TYPE_ID = 2;
    u_int64_t TICK_NUMBER;
    u_int8_t NUMBER_OF_MOTORS;
    MOTOR_DATA M_DATA[10];
    int8_t BLANK_BYTES[25];

};

struct VideoFramePacket{
    u_int8_t FRAME_TYPE_ID = 4;
    //TODO: libjpeg library to handle jpeg image
};
struct ServoControlPacket{

};

//#pragma pack(pop)

#endif // ROBOTPACKETS_H
