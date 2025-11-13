#include "input/joystick_handler.h"
#include "network/mavlink_sender.h"
#include "common/config_loader.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

volatile bool running = true;

double get_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void signal_handler(int signum) {
    (void)signum;
    running = false;
}

int main(int argc, char* argv[]) {
    printf("=== Vepryk Joystick Control ===\n");
    
    // Setup signal handler for clean exit
    signal(SIGINT, signal_handler);
    
    // Load configuration
    VeprykConfig config;
    const char* config_file = (argc > 1) ? argv[1] : "config.json";
    load_vepryk_config(config_file, config);
    
    // Command line overrides
    const char* host = config.network.host.c_str();
    int port = config.network.port;
    
    if (argc > 2) host = argv[2];
    if (argc > 3) port = atoi(argv[3]);
    
    printf("Network: %s:%d\n", host, port);
    printf("SSH: %s@%s:%d\n", config.network.ssh_user.c_str(), 
           config.network.ssh_host.c_str(), config.network.ssh_port);
    printf("RC Values: min=%d, neutral=%d, max=%d\n", 
           config.rc_values.min, config.rc_values.neutral, config.rc_values.max);
    printf("Gear Bits: up=%d, down=%d\n", config.gear_bits.gear_up, config.gear_bits.gear_down);
    printf("\n");
    
    // Initialize input
    JoystickHandler joystick;
    if (!joystick.init()) {
        fprintf(stderr, "Failed to initialize joystick\n");
        return 1;
    }
    
    // Initialize network with config values
    MAVLinkSender sender(config.rc_values, config.gear_bits);
    if (!sender.connect(host, port)) {
        fprintf(stderr, "Failed to connect to %s:%d\n", host, port);
        fprintf(stderr, "Is SSH tunnel running? ssh -L %d:localhost:%d %s@%s\n",
                port, port, config.network.ssh_user.c_str(), config.network.ssh_host.c_str());
        return 1;
    }
    
    printf("Connected!\n");
    printf("Press Ctrl+C to exit.\n\n");
    
    double last_heartbeat = get_time();
    
    while (running) {
        // Update joystick state
        joystick.update(); // Don't need return value
        
        // Get current state and send
        ControlState state = joystick.getState();
        sender.sendControlState(state);
        
        // Send heartbeat every second
        double now = get_time();
        if (now - last_heartbeat > 1.0) {
            sender.sendHeartbeat();
            last_heartbeat = now;
        }
        
        usleep(20000); // 50 Hz (20ms)
    }
    
    printf("\nShutting down...\n");
    joystick.cleanup();
    sender.disconnect();
    
    return 0;
}
