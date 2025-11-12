#include "joystick_handler.h"
#include <stdio.h>
#include <SDL2/SDL.h>

int main(int argc, char* argv[]) {
    printf("Joystick Test Program\n");
    
    JoystickHandler handler;
    
    // Load config file
    const char* config_file = (argc > 1) ? argv[1] : "config.json";
    printf("Loading config from: %s\n", config_file);
    handler.loadConfig(config_file);
    
    if (!handler.init()) {
        printf("ERROR: Failed to initialize joystick\n");
        return 1;
    }
    
    printf("\nMove joystick and press buttons. Press Ctrl+C to quit.\n\n");
    
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