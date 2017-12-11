#ifndef UDPSOCKET_H
#define UDPSOCKET_H
#include <QUdpSocket>
#include <cstddef>
#include <functional>

class UdpSocket : private QUdpSocket
{
    Q_OBJECT
public:
    explicit UdpSocket(QObject* parent, const char* const address);
    virtual ~UdpSocket();
    bool hasPackets() const;
    int64_t receivePacket(char* data);
    int64_t transmitPacket(const char*data, const int64_t len);
    void connectToRobot();
    void disconnectFromRobot();
    void setCallbacks(std::function<void(void)> packetNotif, std::function<void(void)> startTimer);
    void removeCallbacks() const;
public slots:
    void notificateClient();
    void notificateTimer();
private:
    QHostAddress* robotAddress;
    std::function<void(void)> packetNotif;
    std::function<void(void)> startTime;
    quint16 PORT = 10000;
    const int MAX_LEN = 1000;
};

#endif // UDPSOCKET_H
