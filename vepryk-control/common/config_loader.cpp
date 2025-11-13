#include "config_loader.h"
#include <fstream>
#include <sstream>
#include <cstring>

// Simple JSON parser for our limited needs
// Extracts string value: "key": "value"
static bool extract_string(const std::string& json, const char* key, std::string& value) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos = json.find('"', pos + search.length());
    if (pos == std::string::npos) return false;
    pos++; // Skip opening quote
    
    size_t end = json.find('"', pos);
    if (end == std::string::npos) return false;
    
    value = json.substr(pos, end - pos);
    return true;
}

// Extracts integer value: "key": 123
static bool extract_int(const std::string& json, const char* key, int& value) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos += search.length();
    // Skip whitespace
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    
    std::string num_str;
    while (pos < json.length() && (json[pos] >= '0' && json[pos] <= '9')) {
        num_str += json[pos++];
    }
    
    if (num_str.empty()) return false;
    value = atoi(num_str.c_str());
    return true;
}

// Extracts boolean value: "key": true/false
static bool extract_bool(const std::string& json, const char* key, bool& value) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos += search.length();
    // Skip whitespace
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    
    if (json.substr(pos, 4) == "true") {
        value = true;
        return true;
    } else if (json.substr(pos, 5) == "false") {
        value = false;
        return true;
    }
    return false;
}

// Extracts float value: "key": 0.5
static bool extract_float(const std::string& json, const char* key, float& value) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos += search.length();
    // Skip whitespace
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    
    std::string num_str;
    while (pos < json.length() && 
           ((json[pos] >= '0' && json[pos] <= '9') || json[pos] == '.')) {
        num_str += json[pos++];
    }
    
    if (num_str.empty()) return false;
    value = atof(num_str.c_str());
    return true;
}

bool load_vepryk_config(const char* config_path, VeprykConfig& config) {
    // Default values
    config.network.host = "127.0.0.1";
    config.network.port = 14550;
    config.network.ssh_user = "pi";
    config.network.ssh_host = "raspberry.local";
    config.network.ssh_port = 22;
    
    config.rc_values.min = 1000;
    config.rc_values.neutral = 1500;
    config.rc_values.max = 2000;
    
    config.gear_bits.gear_up = 11;
    config.gear_bits.gear_down = 12;
    
    // Joystick defaults
    config.joystick.throttle_axis = 1;
    config.joystick.steering_axis = 3;
    config.joystick.gear_up_button = 4;
    config.joystick.gear_down_button = 5;
    config.joystick.arm_button = 7;
    config.joystick.disarm_button = 6;
    config.joystick.deadzone = 0.5f;
    config.joystick.invert_throttle = true;
    config.joystick.invert_steering = false;
    config.joystick.discrete_values = true;
    
    std::ifstream file(config_path);
    if (!file.is_open()) {
        fprintf(stderr, "Warning: Could not open %s, using defaults\n", config_path);
        return false;
    }
    
    // Read entire file
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json = buffer.str();
    
    // Extract network values
    extract_string(json, "host", config.network.host);
    extract_int(json, "port", config.network.port);
    extract_string(json, "ssh_user", config.network.ssh_user);
    extract_string(json, "ssh_host", config.network.ssh_host);
    extract_int(json, "ssh_port", config.network.ssh_port);
    
    // Extract RC values
    extract_int(json, "min", config.rc_values.min);
    extract_int(json, "neutral", config.rc_values.neutral);
    extract_int(json, "max", config.rc_values.max);
    
    // Extract gear bits
    extract_int(json, "gear_up", config.gear_bits.gear_up);
    extract_int(json, "gear_down", config.gear_bits.gear_down);
    
    // Extract joystick config
    extract_int(json, "throttle_axis", config.joystick.throttle_axis);
    extract_int(json, "steering_axis", config.joystick.steering_axis);
    extract_int(json, "gear_up_button", config.joystick.gear_up_button);
    extract_int(json, "gear_down_button", config.joystick.gear_down_button);
    extract_int(json, "arm_button", config.joystick.arm_button);
    extract_int(json, "disarm_button", config.joystick.disarm_button);
    extract_float(json, "deadzone", config.joystick.deadzone);
    extract_bool(json, "invert_throttle", config.joystick.invert_throttle);
    extract_bool(json, "invert_steering", config.joystick.invert_steering);
    extract_bool(json, "discrete_values", config.joystick.discrete_values);
    
    return true;
}
