#include "ssh_bridge.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <cerrno>

SSHBridge::SSHBridge() : sockfd(-1), connected(false) {}

SSHBridge::~SSHBridge() {
    disconnect();
}

bool SSHBridge::connect(const char* host, int port) {
    // Close existing connection if any
    if (sockfd >= 0) {
        disconnect();
    }
    
    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        return false;
    }
    
    // Enable TCP_NODELAY for low latency (disable Nagle's algorithm)
    int flag = 1;
    if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
        perror("setsockopt TCP_NODELAY failed");
        close(sockfd);
        sockfd = -1;
        return false;
    }
    
    // Enable SO_KEEPALIVE to detect dead connections
    flag = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag)) < 0) {
        perror("setsockopt SO_KEEPALIVE failed");
        // Non-fatal, continue
    }
    
    // Prepare server address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host, &server_addr.sin_addr) <= 0) {
        fprintf(stderr, "Invalid address: %s\n", host);
        close(sockfd);
        sockfd = -1;
        return false;
    }
    
    // Connect to server
    if (::connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        fprintf(stderr, "Connection to %s:%d failed: %s\n", host, port, strerror(errno));
        fprintf(stderr, "Is SSH tunnel running? Try: ssh -L %d:localhost:%d pi@raspberry.local\n", 
                port, port);
        close(sockfd);
        sockfd = -1;
        return false;
    }
    
    // Set non-blocking mode for receive operations
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        perror("fcntl F_GETFL failed");
        close(sockfd);
        sockfd = -1;
        return false;
    }
    
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("fcntl F_SETFL O_NONBLOCK failed");
        close(sockfd);
        sockfd = -1;
        return false;
    }
    
    connected = true;
    printf("SSH Bridge connected to %s:%d\n", host, port);
    return true;
}

int SSHBridge::send(const uint8_t* data, size_t len) {
    if (!connected || sockfd < 0) {
        fprintf(stderr, "Cannot send: not connected\n");
        return -1;
    }
    
    if (data == nullptr || len == 0) {
        fprintf(stderr, "Cannot send: invalid data or length\n");
        return -1;
    }
    
    int sent = ::send(sockfd, data, len, 0);
    if (sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Socket buffer full, try again later
            return 0;
        }
        fprintf(stderr, "Send failed: %s\n", strerror(errno));
        connected = false;
        return -1;
    }
    
    return sent;
}

int SSHBridge::receive(uint8_t* buffer, size_t max_len) {
    if (!connected || sockfd < 0) {
        return -1;
    }
    
    if (buffer == nullptr || max_len == 0) {
        fprintf(stderr, "Cannot receive: invalid buffer or length\n");
        return -1;
    }
    
    int received = recv(sockfd, buffer, max_len, 0);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available (non-blocking mode)
            return 0;
        }
        fprintf(stderr, "Receive failed: %s\n", strerror(errno));
        connected = false;
        return -1;
    }
    
    if (received == 0) {
        // Connection closed by remote
        fprintf(stderr, "Connection closed by remote host\n");
        connected = false;
        return -1;
    }
    
    return received;
}

bool SSHBridge::isConnected() const {
    return connected && sockfd >= 0;
}

void SSHBridge::disconnect() {
    if (sockfd >= 0) {
        printf("Disconnecting SSH bridge...\n");
        close(sockfd);
        sockfd = -1;
    }
    connected = false;
}
