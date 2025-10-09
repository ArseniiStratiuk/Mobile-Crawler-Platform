#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>     // For read(), write(), close(), sleep()
#include <fcntl.h>      // For file control options
#include <termios.h>    // For serial port configuration
#include <errno.h>      // For error number definitions

#include "common/mavlink.h"

#define SERIAL_PORT "/dev/ttyS0" 
#define BAUD_RATE B57600
#define SYSTEM_ID 255
#define COMPONENT_ID 1

int main() {
    int serial_fd;
    
    // OPEN AND CONFIGURE THE SERIAL PORT
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_fd);
        return 1;
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
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1-second read timeout
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_fd);
        return 1;
    }
    
    printf("Serial port configured. Starting to send heartbeats...\n");

    // INFINITE LOOP TO SEND HEARTBEATS
    while (1) {
        // Prepare MAVLink HEARTBEAT message
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        
        mavlink_msg_heartbeat_pack(
            SYSTEM_ID, 
            COMPONENT_ID, 
            &msg, 
            MAV_TYPE_GCS, 
            MAV_AUTOPILOT_GENERIC, 
            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 
            0, 
            MAV_STATE_ACTIVE
        );
        
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        // Send the message
        ssize_t bytes_written = write(serial_fd, buf, len);
        if (bytes_written > 0) {
            printf("Heartbeat sent (%zd bytes).\n", bytes_written);
        } else {
            perror("Failed to write to serial port");
        }

        // Wait for 1 second before sending the next one
        sleep(1);
    }


    close(serial_fd);
    return 0;
}
