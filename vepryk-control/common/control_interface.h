#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include <stdint.h>

struct ControlState {
    // Movement controls (MAVLink RC_CHANNELS_OVERRIDE) (sometimes dead zone of 70!)
    uint16_t throttle;    // 1000-2000, 1500=neutral, >1500=forward, <1500=reverse
    uint16_t steering;    // 1000-2000, 1500=neutral, >1500=right, <1500=left
    
    // Gear controls (MAVLink MANUAL_CONTROL buttons field)
    uint16_t buttons;     // Bitmask: bit11=gear_up(2048), bit12=gear_down(4096)
    
    // System commands (MAVLink COMMAND_LONG)
    bool arm_requested;   // True = send ARM command once
    bool disarm_requested; // True = send DISARM command once
    
    // Metadata
    uint64_t timestamp_ms; // For latency measurement
};

// Initialize to safe defaults
static inline void control_state_init(ControlState* state) {
    state->throttle = 1500;
    state->steering = 1500;
    state->buttons = 0;
    state->arm_requested = false;
    state->disarm_requested = false;
    state->timestamp_ms = 0;
}

#endif
