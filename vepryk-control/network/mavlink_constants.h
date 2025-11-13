#ifndef MAVLINK_CONSTANTS_H
#define MAVLINK_CONSTANTS_H

/**
 * MAVLink Constants - System-Level Configuration
 * 
 * These are system-level constants that rarely change.
 * 
 * PWM values (min/neutral/max) and gear button bits are loaded from config.json
 * at runtime, allowing changes without recompilation.
 * 
 * See config.json for configurable values:
 *   - rc_values: {min, neutral, max} for all PWM controls
 *   - gear_bits: {gear_up, gear_down} for button mapping
 */

// System IDs
#define MAVLINK_SYSTEM_ID       255    // GCS (Ground Control Station)
#define MAVLINK_COMPONENT_ID    1      // Component ID
#define MAVLINK_TARGET_SYSTEM   1      // Rover system ID (detected from heartbeat)
#define MAVLINK_TARGET_COMPONENT 1     // Rover component ID

// RC Channel Numbers (1-based)
#define RC_CHANNEL_STEERING     1      // Steering control
#define RC_CHANNEL_THROTTLE     3      // Throttle/Movement control
#define RC_CHANNEL_GEAR         6      // Gear shift control

// Control Timing
#define GEAR_COMMAND_DURATION   0.51   // Seconds to hold gear command
#define NO_OVERRIDE             65535  // Value indicating channel not overridden
#define HEARTBEAT_INTERVAL      1.0    // Seconds between heartbeats
#define MESSAGE_CHECK_INTERVAL  0.1    // Seconds between message checks
#define CONTROL_SEND_INTERVAL   0.1    // Seconds between control updates

// Connection
#define HEARTBEAT_TIMEOUT       30.0   // Seconds to wait for initial heartbeat
#define ARM_TIMEOUT             5.0    // Seconds to wait for ARM/DISARM ACK

#endif // MAVLINK_CONSTANTS_H
