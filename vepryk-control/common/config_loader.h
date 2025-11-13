#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>

struct NetworkConfig {
    std::string host;
    int port;
    std::string ssh_user;
    std::string ssh_host;
    int ssh_port;
};

struct RCValues {
    int min;
    int neutral;
    int max;
};

struct GearBits {
    int gear_up;
    int gear_down;
};

struct JoystickControlConfig {
    int throttle_axis;
    int steering_axis;
    int gear_up_button;
    int gear_down_button;
    int arm_button;
    int disarm_button;
    float deadzone;
    bool invert_throttle;
    bool invert_steering;
    bool discrete_values;
};

struct VeprykConfig {
    NetworkConfig network;
    RCValues rc_values;
    GearBits gear_bits;
    JoystickControlConfig joystick;
};

// Load complete configuration from JSON file
// Returns true on success, false on failure (uses defaults)
bool load_vepryk_config(const char* config_path, VeprykConfig& config);

#endif // CONFIG_LOADER_H
