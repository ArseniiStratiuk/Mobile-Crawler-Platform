#ifndef MAVLINK_THREAD_H
#define MAVLINK_THREAD_H

#include <QThread>
#include "../network/ssh_bridge.h"
#include "../../headers/common/mavlink.h"

class MainWindow;

class MAVLinkThread : public QThread {
    Q_OBJECT

public:
    MAVLinkThread(MainWindow *window, SSHBridge *bridge);
    ~MAVLinkThread();

protected:
    void run() override;

private:
    void handleHeartbeat(const mavlink_message_t &msg);
    void handleRCChannels(const mavlink_message_t &msg);
    void handleEstimatorStatus(const mavlink_message_t &msg);
    void handleBatteryStatus(const mavlink_message_t &msg);
    void handleAttitude(const mavlink_message_t &msg);
    void handleGlobalPosition(const mavlink_message_t &msg);
    
    MainWindow *window;
    SSHBridge *bridge;
    volatile bool running;
};

#endif
