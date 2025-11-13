#ifndef SSH_BRIDGE_H
#define SSH_BRIDGE_H

#include <stdbool.h>
#include <cstdint>
#include <cstddef>

class SSHBridge {
public:
    SSHBridge();
    ~SSHBridge();
    
    /**
     * Connect to specified host and port
     * 
     * @param host IP address or hostname
     * @param port Port number
     * @return true on success, false on failure
     */
    bool connect(const char* host, int port);
    
    /**
     * Send raw bytes through the connection
     * 
     * @param data Pointer to data buffer
     * @param len Number of bytes to send
     * @return Number of bytes sent, 0 if would block, -1 on error
     */
    int send(const uint8_t* data, size_t len);
    
    /**
     * Receive raw bytes from the connection (non-blocking)
     * 
     * @param buffer Pointer to receive buffer
     * @param max_len Maximum number of bytes to receive
     * @return Number of bytes received, 0 if no data available, -1 on error
     */
    int receive(uint8_t* buffer, size_t max_len);
    
    /**
     * Check if connection is alive
     * 
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
    
    /**
     * Close the connection
     */
    void disconnect();
    
private:
    int sockfd;      // Socket file descriptor
    bool connected;  // Connection state flag
};

#endif
