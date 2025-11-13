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
    
    return true;
}
