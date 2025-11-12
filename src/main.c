#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>     // For read(), write(), close(), usleep()
#include <fcntl.h>      // For file control options
#include <termios.h>    // For serial port configuration
#include <errno.h>      // For error number definitions
#include <sys/time.h>   // For gettimeofday()
#include <sys/select.h> // For select()
#include <ctype.h>      // For tolower()
#include <stdbool.h>    // For bool type

#include "common/mavlink.h"

// Serial port configuration
#define SERIAL_PORT "/dev/ttyS0" 
#define BAUD_RATE B57600
#define SYSTEM_ID 255
#define COMPONENT_ID 1

// RC Channels
#define STEER_CHANNEL 1
#define REVERSE_CHANNEL 3
#define GEAR_CHANNEL 6
#define MOVING_CHANNEL 3

// PWM values for steering
#define STEER_LEFT 1000
#define STEER_RIGHT 2000
#define STEER_NEUTRAL 1500

// PWM values for gears
#define GEAR_NEUTRAL 1500
#define GEAR_UP 2000
#define GEAR_DOWN 1000

// PWM values for movement
#define FORWARD_MOVEMENT 2000
#define REVERSE_MOVEMENT 1000
#define NEUTRAL_MOVEMENT 1500

// Timing
#define SENDING_TIME 0.51
#define NO_OVERRIDE 65535

// Controller state structure
typedef struct {
    int serial_fd;
    uint8_t target_system;
    uint8_t target_component;
    
    // Current state
    float current_gear;
    bool gear_known;
    bool running;
    uint16_t steer_pwm;
    uint16_t drive_or_reverse;
    
    // Timers
    double last_heartbeat_time;
    double last_message_check_time;
    double last_control_send_time;
    
    // Gear command state
    bool gear_command_active;
    uint8_t gear_command_channel;
    uint16_t gear_command_value;
    double gear_command_end_time;
} RoverController;

// Function prototypes
double get_time();
int configure_serial_port(const char* port);
bool wait_for_heartbeat(RoverController* ctrl);
bool send_mavlink_message(RoverController* ctrl, mavlink_message_t* msg);
bool arm_vehicle(RoverController* ctrl);
void disarm_vehicle(RoverController* ctrl);
void send_heartbeat(RoverController* ctrl);
void check_messages(RoverController* ctrl);
void send_control_signals(RoverController* ctrl);
void start_gear_change(RoverController* ctrl, uint8_t channel, uint16_t value, double duration);
void gear_up(RoverController* ctrl);
void gear_down(RoverController* ctrl);
void process_command(RoverController* ctrl, const char* cmd);
void print_help();
int check_stdin_ready();
void run_controller(RoverController* ctrl);

// Get current time in seconds (with decimal precision)
double get_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

// Configure serial port
int configure_serial_port(const char* port) {
    int serial_fd = open(port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_fd);
        return -1;
    }
    
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1 second read timeout
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_fd);
        return -1;
    }
    
    return serial_fd;
}

// Wait for vehicle heartbeat
bool wait_for_heartbeat(RoverController* ctrl) {
    printf("Waiting for vehicle heartbeat...\n");
    
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    double start_time = get_time();
    double timeout = 30.0; // 30 second timeout
    
    while (get_time() - start_time < timeout) {
        ssize_t bytes_read = read(ctrl->serial_fd, buf, sizeof(buf));
        
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        ctrl->target_system = msg.sysid;
                        ctrl->target_component = msg.compid;
                        printf("Vehicle connected! System ID: %d, Component ID: %d\n", 
                               ctrl->target_system, ctrl->target_component);
                        return true;
                    }
                }
            }
        }
        
        usleep(10000); // 10ms delay
    }
    
    printf("Timeout waiting for vehicle heartbeat.\n");
    return false;
}

// Send MAVLink message
bool send_mavlink_message(RoverController* ctrl, mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    
    ssize_t bytes_written = write(ctrl->serial_fd, buf, len);
    if (bytes_written < 0) {
        perror("Failed to write to serial port");
        return false;
    }
    
    return true;
}

// Arm the vehicle
bool arm_vehicle(RoverController* ctrl) {
    printf("Attempting to ARM...\n");
    
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        ctrl->target_system,
        ctrl->target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,   // confirmation
        1.0, // param1: 1 to arm
        0, 0, 0, 0, 0, 0
    );
    
    if (!send_mavlink_message(ctrl, &msg)) {
        return false;
    }
    
    // Wait for ACK
    double start_time = get_time();
    double timeout = 5.0;
    
    mavlink_message_t ack_msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    while (get_time() - start_time < timeout) {
        ssize_t bytes_read = read(ctrl->serial_fd, buf, sizeof(buf));
        
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &ack_msg, &status)) {
                    if (ack_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&ack_msg, &ack);
                        
                        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                            if (ack.result == MAV_RESULT_ACCEPTED) {
                                printf("ARMED successfully!\n");
                                return true;
                            } else {
                                printf("Failed to ARM! Result: %d\n", ack.result);
                                return false;
                            }
                        }
                    }
                }
            }
        }
        
        usleep(10000); // 10ms delay
    }
    
    printf("Timeout waiting for ARM acknowledgment.\n");
    return false;
}

// Disarm the vehicle
void disarm_vehicle(RoverController* ctrl) {
    printf("Attempting to DISARM...\n");
    
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        ctrl->target_system,
        ctrl->target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,   // confirmation
        0.0, // param1: 0 to disarm
        0, 0, 0, 0, 0, 0
    );
    
    send_mavlink_message(ctrl, &msg);
    
    // Wait a moment for response
    usleep(500000); // 0.5 second
    
    printf("DISARMED.\n");
}

// Send heartbeat
void send_heartbeat(RoverController* ctrl) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0, 0, 0
    );
    
    send_mavlink_message(ctrl, &msg);
    ctrl->last_heartbeat_time = get_time();
}

// Check incoming messages
void check_messages(RoverController* ctrl) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    ssize_t bytes_read = read(ctrl->serial_fd, buf, sizeof(buf));
    
    if (bytes_read > 0) {
        for (ssize_t i = 0; i < bytes_read; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
                    mavlink_estimator_status_t estimator;
                    mavlink_msg_estimator_status_decode(&msg, &estimator);
                    
                    if (!ctrl->gear_known || estimator.vel_ratio != ctrl->current_gear) {
                        if (ctrl->gear_known) {
                            printf("\nGEAR UPDATE: %.2f -> %.2f\n", 
                                   ctrl->current_gear, estimator.vel_ratio);
                        }
                        ctrl->current_gear = estimator.vel_ratio;
                        ctrl->gear_known = true;
                    }
                }
            }
        }
    }
    
    ctrl->last_message_check_time = get_time();
}

// Send control signals
void send_control_signals(RoverController* ctrl) {
    uint16_t overrides[18];
    
    // Initialize all channels to NO_OVERRIDE
    for (int i = 0; i < 18; i++) {
        overrides[i] = NO_OVERRIDE;
    }
    
    // Set steering channel (channel 1, index 0)
    overrides[STEER_CHANNEL - 1] = ctrl->steer_pwm;
    
    // Set movement channel (channel 3, index 2)
    overrides[MOVING_CHANNEL - 1] = ctrl->drive_or_reverse;
    
    // Handle gear command
    if (ctrl->gear_command_active) {
        if (get_time() < ctrl->gear_command_end_time) {
            overrides[ctrl->gear_command_channel - 1] = ctrl->gear_command_value;
        } else {
            // Time expired, send neutral and complete
            overrides[ctrl->gear_command_channel - 1] = GEAR_NEUTRAL;
            printf("Gear command complete. Channel %d reset to %d\n", 
                   ctrl->gear_command_channel, GEAR_NEUTRAL);
            ctrl->gear_command_active = false;
        }
    }
    
    // Send RC_CHANNELS_OVERRIDE message
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        ctrl->target_system,
        ctrl->target_component,
        overrides[0], overrides[1], overrides[2], overrides[3],
        overrides[4], overrides[5], overrides[6], overrides[7],
        overrides[8], overrides[9], overrides[10], overrides[11],
        overrides[12], overrides[13], overrides[14], overrides[15],
        overrides[16], overrides[17]
    );
    
    send_mavlink_message(ctrl, &msg);
    ctrl->last_control_send_time = get_time();
}

// Start gear change
void start_gear_change(RoverController* ctrl, uint8_t channel, uint16_t value, double duration) {
    printf("Starting gear change: Channel %d, PWM %d, Duration %.2fs\n", 
           channel, value, duration);
    ctrl->gear_command_active = true;
    ctrl->gear_command_channel = channel;
    ctrl->gear_command_value = value;
    ctrl->gear_command_end_time = get_time() + duration;
}

// Gear up
void gear_up(RoverController* ctrl) {
    printf("Shifting GEAR UP\n");
    start_gear_change(ctrl, GEAR_CHANNEL, GEAR_UP, SENDING_TIME);
}

// Gear down
void gear_down(RoverController* ctrl) {
    printf("Shifting GEAR DOWN\n");
    
    if (!ctrl->gear_known) {
        printf("Current gear unknown. Sending gear down signal...\n");
        start_gear_change(ctrl, GEAR_CHANNEL, GEAR_DOWN, SENDING_TIME);
    } else {
        printf("Current gear %.2f. Lowering gear on Channel 6...\n", ctrl->current_gear);
        start_gear_change(ctrl, GEAR_CHANNEL, GEAR_DOWN, SENDING_TIME);
    }
}

// Process user command
void process_command(RoverController* ctrl, const char* cmd) {
    if (strcmp(cmd, "e") == 0) {
        gear_up(ctrl);
    } else if (strcmp(cmd, "q") == 0) {
        gear_down(ctrl);
    } else if (strcmp(cmd, "a") == 0) {
        printf("Turning LEFT\n");
        ctrl->steer_pwm = STEER_LEFT;
    } else if (strcmp(cmd, "d") == 0) {
        printf("Turning RIGHT\n");
        ctrl->steer_pwm = STEER_RIGHT;
    } else if (strcmp(cmd, "s") == 0) {
        printf("Steering NEUTRAL\n");
        ctrl->steer_pwm = STEER_NEUTRAL;
    } else if (strcmp(cmd, "gear") == 0) {
        if (ctrl->gear_known) {
            printf("Current reported gear: %.2f\n", ctrl->current_gear);
        } else {
            printf("Current reported gear: unknown\n");
        }
    } else if (strcmp(cmd, "quit") == 0) {
        printf("Quitting...\n");
        ctrl->running = false;
    } else if (strcmp(cmd, "w") == 0) {
        ctrl->drive_or_reverse = FORWARD_MOVEMENT;
        printf("Moving forward...\n");
    } else if (strcmp(cmd, "r") == 0) {
        ctrl->drive_or_reverse = REVERSE_MOVEMENT;
        printf("Moving reverse...\n");
    } else if (strcmp(cmd, "x") == 0) {
        ctrl->drive_or_reverse = NEUTRAL_MOVEMENT;
        printf("Neutral moving...\n");
    } else {
        printf("Unknown command\n");
    }
}

// Print help menu
void print_help() {
    printf("\n--- Rover Control Ready ---\n");
    printf(" e = Gear Up\n");
    printf(" q = Gear Down\n");
    printf(" a = Steer Left\n");
    printf(" d = Steer Right\n");
    printf(" s = Steer Neutral\n");
    printf(" gear = Show current gear\n");
    printf(" quit = Disarm and Exit\n");
    printf(" w = move forward\n");
    printf(" r = move backward\n");
    printf(" x = stop moving\n");
    printf("---------------------------\n\n");
}

// Check if stdin has data ready (non-blocking)
int check_stdin_ready() {
    fd_set readfds;
    struct timeval tv;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0;
}

// Main controller loop
void run_controller(RoverController* ctrl) {
    if (!arm_vehicle(ctrl)) {
        ctrl->running = false;
        return;
    }
    
    print_help();
    printf("Target: sys=%d, comp=%d\n\n", ctrl->target_system, ctrl->target_component);
    
    char input_buffer[256];
    
    while (ctrl->running) {
        double current_time = get_time();
        
        // 1. Send heartbeat every 1 second
        if (current_time - ctrl->last_heartbeat_time >= 1.0) {
            send_heartbeat(ctrl);
        }
        
        // 2. Check incoming messages every 0.1 seconds
        if (current_time - ctrl->last_message_check_time >= 0.1) {
            check_messages(ctrl);
        }
        
        // 3. Send control signals every 0.1 seconds
        if (current_time - ctrl->last_control_send_time >= 0.1) {
            send_control_signals(ctrl);
        }
        
        // 4. Check for user input (non-blocking)
        if (check_stdin_ready()) {
            if (fgets(input_buffer, sizeof(input_buffer), stdin) != NULL) {
                // Remove trailing newline
                input_buffer[strcspn(input_buffer, "\n")] = 0;
                
                // Convert to lowercase
                for (int i = 0; input_buffer[i]; i++) {
                    input_buffer[i] = tolower(input_buffer[i]);
                }
                
                // Trim leading/trailing whitespace
                char* cmd = input_buffer;
                while (*cmd == ' ' || *cmd == '\t') cmd++;
                
                if (strlen(cmd) > 0) {
                    process_command(ctrl, cmd);
                }
            }
        }
        
        // Sleep for 50ms
        usleep(50000);
    }
    
    disarm_vehicle(ctrl);
    printf("Controller stopped. Robot disarmed.\n");
}

int main() {
    printf("Connecting to %s at 57600 baud...\n", SERIAL_PORT);
    
    // Configure serial port
    int serial_fd = configure_serial_port(SERIAL_PORT);
    if (serial_fd < 0) {
        return 1;
    }
    
    printf("Serial port configured.\n");
    
    // Initialize controller state
    RoverController ctrl = {
        .serial_fd = serial_fd,
        .target_system = 0,
        .target_component = 0,
        .current_gear = 0.0,
        .gear_known = false,
        .running = true,
        .steer_pwm = STEER_NEUTRAL,
        .drive_or_reverse = NEUTRAL_MOVEMENT,
        .last_heartbeat_time = get_time(),
        .last_message_check_time = get_time(),
        .last_control_send_time = get_time(),
        .gear_command_active = false,
        .gear_command_channel = 0,
        .gear_command_value = 0,
        .gear_command_end_time = 0.0
    };
    
    // Wait for vehicle heartbeat
    if (!wait_for_heartbeat(&ctrl)) {
        close(serial_fd);
        return 1;
    }
    
    // Run the main control loop
    run_controller(&ctrl);
    
    close(serial_fd);
    return 0;
}
