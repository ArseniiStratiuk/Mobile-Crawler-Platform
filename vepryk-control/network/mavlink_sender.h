#ifndef MAVLINK_SENDER_H
#define MAVLINK_SENDER_H

#include "../common/control_interface.h"
#include "../common/config_loader.h"
#include "ssh_bridge.h"
#include <stdint.h>

// Forward declare MAVLink types
struct __mavlink_message;

class MAVLinkSender {
public:
    MAVLinkSender();
    MAVLinkSender(const RCValues& rc_vals, const GearBits& gear_bits);
    ~MAVLinkSender();
    
    // Connect to Pi via SSH tunnel
    // Waits for vehicle heartbeat and detects target system/component
    bool connect(const char* host, int port);
    
    // Send control state as MAVLink messages
    // Handles RC_CHANNELS_OVERRIDE for steering/throttle
    // Handles gear changes via timed channel overrides
    // Handles ARM/DISARM commands
    bool sendControlState(const ControlState& state);
    
    // Send heartbeat (call every 1 second)
    bool sendHeartbeat();
    
    // Check connection status
    bool isConnected() const;
    
    // Disconnect
    void disconnect();
    
    // Access to bridge for receiver thread
    SSHBridge* getBridge() { return &bridge; }
    
    // Get current gear (from ESTIMATOR_STATUS)
    float getCurrentGear() const { return current_gear; }
    bool isGearKnown() const { return gear_known; }
    
private:
    SSHBridge bridge;
    
    // MAVLink system IDs
    uint8_t source_system;     // 255 (GCS)
    uint8_t target_system;     // Detected from heartbeat
    uint8_t target_component;  // Detected from heartbeat (not used in header before)
    
    // Track previous state for edge detection
    ControlState prev_state;
    
    // Gear command state (matches main.c implementation)
    bool gear_command_active;
    uint8_t gear_command_channel;
    uint16_t gear_command_value;
    double gear_command_end_time;
    
    // Current gear tracking (from ESTIMATOR_STATUS messages)
    float current_gear;
    bool gear_known;
    
    // Timing
    double last_message_check_time;
    
    // Config values (from config.json)
    RCValues rc_values;
    GearBits gear_bits;
    
    // Helper methods
    double get_time();
    bool wait_for_heartbeat();
    bool send_mavlink_message(struct __mavlink_message* msg);
    bool send_rc_channels_override(uint16_t steering, uint16_t throttle);
    bool send_arm_command(bool arm);
    void start_gear_change(uint8_t channel, uint16_t value, double duration);
    void check_messages();
};

#endif
