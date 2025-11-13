#ifndef JOYSTICK_CONFIG_H
#define JOYSTICK_CONFIG_H

// Simple configuration struct for joystick control mapping.
// Values are populated from config.json via config_loader.cpp
struct JoystickConfig {
    // Axis mappings
    int throttle_axis;
    int steering_axis;
    
    // Button mappings
    int gear_up_button;
    int gear_down_button;
    int arm_button;
    int disarm_button;
    
    // Control parameters
    float deadzone;
    bool invert_throttle;
    bool invert_steering;
    bool discrete_values;
    
    // RC values
    int rc_min;
    int rc_neutral;
    int rc_max;
    
    // Gear bit positions
    int gear_up_bit;
    int gear_down_bit;
};

#endif
