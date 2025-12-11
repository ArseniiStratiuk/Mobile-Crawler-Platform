#include "mavlink_thread.h"
#include "main_window.h"
#include <QDateTime>
#include <unistd.h>
#include <cmath>

MAVLinkThread::MAVLinkThread(MainWindow *window, SSHBridge *bridge)
    : window(window), bridge(bridge), running(true) {
}

MAVLinkThread::~MAVLinkThread() {
    running = false;
    wait();
}

void MAVLinkThread::run() {
    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (running) {
        int n = bridge->receive(buf, sizeof(buf));
        if (n <= 0) {
            usleep(10000); // 10ms
            continue;
        }
        
        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        handleHeartbeat(msg);
                        break;
                    case MAVLINK_MSG_ID_RC_CHANNELS:
                        handleRCChannels(msg);
                        break;
                    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
                        handleEstimatorStatus(msg);
                        break;
                    case MAVLINK_MSG_ID_BATTERY_STATUS:
                        handleBatteryStatus(msg);
                        break;
                    case MAVLINK_MSG_ID_ATTITUDE:
                        handleAttitude(msg);
                        break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        handleGlobalPosition(msg);
                        break;
                }
            }
        }
    }
}

void MAVLinkThread::handleHeartbeat(const mavlink_message_t &msg) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    window->telemetryData.connected = true;
    window->telemetryData.system_id = msg.sysid;
    window->telemetryData.component_id = msg.compid;
    window->telemetryData.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    window->telemetryData.last_heartbeat_ms = QDateTime::currentMSecsSinceEpoch();
}

void MAVLinkThread::handleRCChannels(const mavlink_message_t &msg) {
    mavlink_rc_channels_t rc;
    mavlink_msg_rc_channels_decode(&msg, &rc);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    if (rc.chancount >= 3) {
        window->telemetryData.throttle = rc.chan1_raw;
        window->telemetryData.steering = rc.chan2_raw;
        window->telemetryData.gear = rc.chan3_raw;
    }
}

void MAVLinkThread::handleEstimatorStatus(const mavlink_message_t &msg) {
    mavlink_estimator_status_t est;
    mavlink_msg_estimator_status_decode(&msg, &est);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    window->telemetryData.actual_gear = est.vel_ratio;
    window->telemetryData.gear_known = true;
}

void MAVLinkThread::handleBatteryStatus(const mavlink_message_t &msg) {
    mavlink_battery_status_t bat;
    mavlink_msg_battery_status_decode(&msg, &bat);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    
    // Calculate voltage from voltages array
    float voltage = 0.0f;
    int cell_count = 0;
    for (int i = 0; i < 10 && bat.voltages[i] != UINT16_MAX; i++) {
        voltage += bat.voltages[i] / 1000.0f;
        cell_count++;
    }
    
    if (cell_count > 0) {
        window->telemetryData.voltage = voltage;
    }
    
    window->telemetryData.battery_remaining = bat.battery_remaining;
}

void MAVLinkThread::handleAttitude(const mavlink_message_t &msg) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    window->telemetryData.roll = att.roll * 180.0f / M_PI;
    window->telemetryData.pitch = att.pitch * 180.0f / M_PI;
    window->telemetryData.yaw = att.yaw * 180.0f / M_PI;
}

void MAVLinkThread::handleGlobalPosition(const mavlink_message_t &msg) {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(&msg, &pos);
    
    std::lock_guard<std::mutex> lock(window->dataMutex);
    
    // vx, vy, vz are in cm/s, convert to m/s
    float vx = pos.vx / 100.0f;
    float vy = pos.vy / 100.0f;
    window->telemetryData.groundspeed = std::sqrt(vx*vx + vy*vy);
}
