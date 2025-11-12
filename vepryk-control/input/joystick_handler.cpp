#include "joystick_handler.h"
#include <SDL2/SDL.h>
#include <stdio.h>

JoystickHandler::JoystickHandler() 
    : joystick(nullptr)
{
    control_state_init(&current_state);
    control_state_init(&previous_state);
}

JoystickHandler::~JoystickHandler() {
    cleanup();
}

bool JoystickHandler::loadConfig(const std::string& config_file) {
    return config.loadFromFile(config_file);
}

bool JoystickHandler::init() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\\n", SDL_GetError());
        return false;
    }
    
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks < 1) {
        fprintf(stderr, "No joysticks detected\\n");
        return false;
    }
    
    joystick = SDL_JoystickOpen(0);
    if (!joystick) {
        fprintf(stderr, "Failed to open joystick: %s\\n", SDL_GetError());
        return false;
    }
    
    printf("Joystick detected: %s\\n", SDL_JoystickName((SDL_Joystick*)joystick));
    printf("  Axes: %d\\n", SDL_JoystickNumAxes((SDL_Joystick*)joystick));
    printf("  Buttons: %d\\n", SDL_JoystickNumButtons((SDL_Joystick*)joystick));
    
    return true;
}

bool JoystickHandler::update() {
    SDL_Event event;
    bool state_changed = false;
    
    previous_state = current_state;
    
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_JOYAXISMOTION) {
            if (event.jaxis.axis == config.throttle_axis) {
                current_state.throttle = mapAxisToRC(event.jaxis.value, config.invert_throttle);
                state_changed = true;
            }
            else if (event.jaxis.axis == config.steering_axis) {
                current_state.steering = mapAxisToRC(event.jaxis.value, config.invert_steering);
                state_changed = true;
            }
        }
        else if (event.type == SDL_JOYBUTTONDOWN) {
            if (event.jbutton.button == config.gear_up_button) {
                current_state.buttons = (1 << config.gear_up_bit);  // Set gear up bit
                state_changed = true;
            }
            else if (event.jbutton.button == config.gear_down_button) {
                current_state.buttons = (1 << config.gear_down_bit);  // Set gear down bit
                state_changed = true;
            }
            else if (event.jbutton.button == config.arm_button) {
                current_state.arm_requested = true;
                state_changed = true;
            }
            else if (event.jbutton.button == config.disarm_button) {
                current_state.disarm_requested = true;
                state_changed = true;
            }
        }
        else if (event.type == SDL_JOYBUTTONUP) {
            // Clear one-shot button flags
            if (event.jbutton.button == config.gear_up_button || 
                event.jbutton.button == config.gear_down_button) {
                current_state.buttons = 0;
            }
            if (event.jbutton.button == config.arm_button) {
                current_state.arm_requested = false;
            }
            if (event.jbutton.button == config.disarm_button) {
                current_state.disarm_requested = false;
            }
        }
    }
    
    return state_changed;
}

ControlState JoystickHandler::getState() const {
    return current_state;
}

void JoystickHandler::cleanup() {
    if (joystick) {
        SDL_JoystickClose((SDL_Joystick*)joystick);
        joystick = nullptr;
    }
    SDL_Quit();
}

int16_t JoystickHandler::mapAxisToRC(int raw_value, bool invert) {
    // SDL axis range: -32768 to +32767
    // RC range: configurable via config (default: 1000-2000)
    
    if (isDeadzone(raw_value)) {
        return config.rc_neutral; // Neutral
    }
    
    // Normalize to -1.0 to +1.0
    float normalized = raw_value / 32768.0f;
    
    // Invert if requested
    if (invert) {
        normalized = -normalized;
    }
    
    if (config.discrete_values) {
        // Snap to discrete values: min, neutral, or max
        if (normalized > config.deadzone) {
            return config.rc_max;  // Full positive (forward/right)
        } else if (normalized < -config.deadzone) {
            return config.rc_min;  // Full negative (backward/left)
        } else {
            return config.rc_neutral;  // Neutral (within deadzone)
        }
    } else {
        // Continuous values with deadzone
        int16_t range = (config.rc_max - config.rc_min) / 2;
        int16_t rc_value = config.rc_neutral + (int16_t)(normalized * range);
        
        // Clamp to valid range
        if (rc_value < config.rc_min) rc_value = config.rc_min;
        if (rc_value > config.rc_max) rc_value = config.rc_max;
        
        return rc_value;
    }
}

bool JoystickHandler::isDeadzone(int raw_value) {
    float normalized = raw_value / 32768.0f;
    return (normalized > -config.deadzone && normalized < config.deadzone);
}