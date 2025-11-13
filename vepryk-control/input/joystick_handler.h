#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#include "../common/control_interface.h"
#include "joystick_config.h"
#include <stdbool.h>
#include <string>

class JoystickHandler {
public:
    JoystickHandler();
    ~JoystickHandler();
    
    // Load configuration from JSON file
    // Call before init() to use custom config
    bool loadConfig(const std::string& config_file);
    
    // Set configuration directly from loaded values
    void setConfig(const JoystickConfig& cfg);
    
    // Initialize SDL2 and open first available joystick
    // Returns: true on success, false if no joystick found
    bool init();
    
    // Poll for new joystick events and update internal state
    // Returns: true if state changed since last call
    bool update();
    
    // Get current control state (safe to call anytime)
    ControlState getState() const;
    
    // Cleanup SDL2 resources
    void cleanup();
    
private:
    void* joystick;        // SDL_Joystick* (opaque to avoid SDL2 in header)
    ControlState current_state;
    ControlState previous_state;
    
    // Configuration loaded from file
    JoystickConfig config;
    
    // Helper methods
    int16_t mapAxisToRC(int raw_value, bool invert = false);
    bool isDeadzone(int raw_value);
};

#endif