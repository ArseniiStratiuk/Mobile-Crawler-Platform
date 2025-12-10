#include "main_window.h"
#include "mavlink_thread.h"
#include "../input/joystick_handler.h"
#include "../network/mavlink_sender.h"
#include "../common/config_loader.h"
#include <QApplication>
#include <QTimer>
#include <signal.h>

volatile bool running = true;

void signal_handler(int signum) {
    (void)signum;
    running = false;
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    printf("=== Vepryk Integrated Control ===\n");
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    
    // Load configuration
    VeprykConfig config;
    const char* config_file = "config.json";
    load_vepryk_config(config_file, config);
    
    const char* host = config.network.host.c_str();
    int port = config.network.port;
    
    printf("Network: %s:%d\n", host, port);
    printf("RC Values: min=%d, neutral=%d, max=%d\n", 
           config.rc_values.min, config.rc_values.neutral, config.rc_values.max);
    printf("\n");
    
    // Initialize joystick
    JoystickHandler joystick;
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
    
    joystick.setConfig(joy_cfg);
    
    if (!joystick.init()) {
        fprintf(stderr, "Failed to initialize joystick\n");
        return 1;
    }
    
    printf("Joystick initialized\n");
    
    // Initialize MAVLink sender
    MAVLinkSender sender(config.rc_values, config.gear_bits);
    if (!sender.connect(host, port)) {
        fprintf(stderr, "Failed to connect to %s:%d\n", host, port);
        fprintf(stderr, "Is SSH tunnel running? ssh -L %d:localhost:%d %s@%s\n",
                port, port, config.network.ssh_user.c_str(), config.network.ssh_host.c_str());
        joystick.cleanup();
        return 1;
    }
    
    printf("MAVLink connected!\n\n");
    
    // Create GUI
    MainWindow window;
    window.show();
    
    // Start MAVLink receiver thread
    MAVLinkThread *mavThread = new MAVLinkThread(&window, sender.getBridge());
    mavThread->start();
    
    // Setup control loop timer (50 Hz)
    QTimer *controlTimer = new QTimer(&window);
    QObject::connect(controlTimer, &QTimer::timeout, [&]() {
        joystick.update();
        ControlState state = joystick.getState();
        sender.sendControlState(state);
        
        // Update GUI with current control values
        std::lock_guard<std::mutex> lock(window.dataMutex);
        window.telemetryData.throttle = state.throttle;
        window.telemetryData.steering = state.steering;
        window.telemetryData.gear = state.buttons;
    });
    controlTimer->start(20); // 50 Hz
    
    // Setup heartbeat timer (1 Hz)
    QTimer *heartbeatTimer = new QTimer(&window);
    QObject::connect(heartbeatTimer, &QTimer::timeout, [&]() {
        sender.sendHeartbeat();
    });
    heartbeatTimer->start(1000); // 1 Hz
    
    // Run application
    int result = app.exec();
    
    // Cleanup
    printf("\nShutting down...\n");
    delete mavThread;
    sender.disconnect();
    joystick.cleanup();
    
    return result;
}
