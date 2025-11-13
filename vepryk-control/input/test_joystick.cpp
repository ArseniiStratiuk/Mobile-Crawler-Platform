#include "joystick_handler.h"
#include "../common/config_loader.h"
#include <stdio.h>
#include <SDL2/SDL.h>

int main(int argc, char* argv[]) {
    printf("Joystick Test Program\n");
    
    // Load global project configuration
    VeprykConfig config;
    const char* config_file = (argc > 1) ? argv[1] : "config.json";
    printf("Loading config from: %s\n", config_file);
    load_vepryk_config(config_file, config);
    
    printf("Joystick Configuration:\n");
    printf("  Axes: throttle=%d, steering=%d\n", 
           config.joystick.throttle_axis, config.joystick.steering_axis);
    printf("  Buttons: gear_up=%d, gear_down=%d, arm=%d, disarm=%d\n",
           config.joystick.gear_up_button, config.joystick.gear_down_button,
           config.joystick.arm_button, config.joystick.disarm_button);
    printf("  Control: deadzone=%.2f, invert_throttle=%d, invert_steering=%d, discrete=%d\n",
           config.joystick.deadzone, config.joystick.invert_throttle,
           config.joystick.invert_steering, config.joystick.discrete_values);
    printf("  RC Range: [%d, %d, %d]\n", 
           config.rc_values.min, config.rc_values.neutral, config.rc_values.max);
    printf("  Gear Bits: up=%d, down=%d\n\n", 
           config.gear_bits.gear_up, config.gear_bits.gear_down);
    
    // Initialize joystick handler with config
    JoystickHandler handler;
    
    // Convert config to JoystickConfig format
    JoystickConfig joy_cfg;
    joy_cfg.throttle_axis = config.joystick.throttle_axis;
    joy_cfg.steering_axis = config.joystick.steering_axis;
    joy_cfg.gear_up_button = config.joystick.gear_up_button;
    joy_cfg.gear_down_button = config.joystick.gear_down_button;
    joy_cfg.arm_button = config.joystick.arm_button;
    joy_cfg.disarm_button = config.joystick.disarm_button;
    joy_cfg.deadzone = config.joystick.deadzone;
    joy_cfg.invert_throttle = config.joystick.invert_throttle;
    joy_cfg.invert_steering = config.joystick.invert_steering;
    joy_cfg.discrete_values = config.joystick.discrete_values;
    joy_cfg.rc_min = config.rc_values.min;
    joy_cfg.rc_neutral = config.rc_values.neutral;
    joy_cfg.rc_max = config.rc_values.max;
    joy_cfg.gear_up_bit = config.gear_bits.gear_up;
    joy_cfg.gear_down_bit = config.gear_bits.gear_down;
    
    handler.setConfig(joy_cfg);
    
    if (!handler.init()) {
        printf("ERROR: Failed to initialize joystick\n");
        return 1;
    }
    
    printf("Move joystick and press buttons. Press Ctrl+C to quit.\n\n");
    
    while (true) {
        if (handler.update()) {
            ControlState state = handler.getState();
            
            printf("\r[Throttle: %4d] [Steering: %4d] [Buttons: 0x%04X] [ARM: %d] [DISARM: %d]  ",
                   state.throttle, state.steering, state.buttons, 
                   state.arm_requested, state.disarm_requested);
            fflush(stdout);
        }
        
        SDL_Delay(20); // 50 Hz
    }
    
    handler.cleanup();
    return 0;
}