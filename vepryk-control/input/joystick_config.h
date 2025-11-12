#ifndef JOYSTICK_CONFIG_H
#define JOYSTICK_CONFIG_H

#include <string>
#include <fstream>
#include <sstream>
#include <cstdio>

struct JoystickConfig {
    // Axis mappings
    int throttle_axis;
    int steering_axis;
    
    // Button mappings
    int gear_up_button;
    int gear_down_button;
    int arm_button;
    int disarm_button;
    
    // Control parameters
    float deadzone;
    bool invert_throttle;
    bool invert_steering;
    bool discrete_values;
    
    // RC values
    int rc_min;
    int rc_neutral;
    int rc_max;
    
    // Gear bit positions
    int gear_up_bit;
    int gear_down_bit;
    
    // Default constructor with safe defaults
    JoystickConfig() 
        : throttle_axis(1)
        , steering_axis(3)
        , gear_up_button(0)
        , gear_down_button(1)
        , arm_button(7)
        , disarm_button(6)
        , deadzone(0.5f)
        , invert_throttle(true)
        , invert_steering(false)
        , discrete_values(true)
        , rc_min(1000)
        , rc_neutral(1500)
        , rc_max(2000)
        , gear_up_bit(11)
        , gear_down_bit(12)
    {}
    
    // Load from JSON file
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            fprintf(stderr, "Warning: Could not open config file '%s', using defaults\n", filename.c_str());
            return false;
        }
        
        std::string line;
        std::string content;
        while (std::getline(file, line)) {
            content += line;
        }
        file.close();
        
        // Simple JSON parser (handles the specific format we need)
        parseJSON(content);
        
        printf("Config loaded from '%s'\n", filename.c_str());
        printConfig();
        return true;
    }
    
    void printConfig() const {
        printf("Joystick Configuration:\n");
        printf("  Axes: throttle=%d, steering=%d\n", throttle_axis, steering_axis);
        printf("  Buttons: gear_up=%d, gear_down=%d, arm=%d, disarm=%d\n", 
               gear_up_button, gear_down_button, arm_button, disarm_button);
        printf("  Control: deadzone=%.2f, invert_throttle=%d, invert_steering=%d, discrete=%d\n",
               deadzone, invert_throttle, invert_steering, discrete_values);
        printf("  RC Range: [%d, %d, %d]\n", rc_min, rc_neutral, rc_max);
        printf("  Gear Bits: up=%d, down=%d\n", gear_up_bit, gear_down_bit);
    }
    
private:
    // Simple JSON value extractor
    int extractInt(const std::string& content, const std::string& key) {
        std::string searchKey = "\"" + key + "\":";
        size_t pos = content.find(searchKey);
        if (pos == std::string::npos) return -1;
        
        pos += searchKey.length();
        while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
        
        int value = 0;
        sscanf(content.c_str() + pos, "%d", &value);
        return value;
    }
    
    float extractFloat(const std::string& content, const std::string& key) {
        std::string searchKey = "\"" + key + "\":";
        size_t pos = content.find(searchKey);
        if (pos == std::string::npos) return -1.0f;
        
        pos += searchKey.length();
        while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
        
        float value = 0.0f;
        sscanf(content.c_str() + pos, "%f", &value);
        return value;
    }
    
    bool extractBool(const std::string& content, const std::string& key) {
        std::string searchKey = "\"" + key + "\":";
        size_t pos = content.find(searchKey);
        if (pos == std::string::npos) return false;
        
        pos += searchKey.length();
        while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
        
        if (content.substr(pos, 4) == "true") return true;
        return false;
    }
    
    void parseJSON(const std::string& content) {
        // Joystick mappings
        int val;
        if ((val = extractInt(content, "throttle_axis")) >= 0) throttle_axis = val;
        if ((val = extractInt(content, "steering_axis")) >= 0) steering_axis = val;
        if ((val = extractInt(content, "gear_up_button")) >= 0) gear_up_button = val;
        if ((val = extractInt(content, "gear_down_button")) >= 0) gear_down_button = val;
        if ((val = extractInt(content, "arm_button")) >= 0) arm_button = val;
        if ((val = extractInt(content, "disarm_button")) >= 0) disarm_button = val;
        
        // Control parameters
        float fval;
        if ((fval = extractFloat(content, "deadzone")) >= 0.0f) deadzone = fval;
        invert_throttle = extractBool(content, "invert_throttle");
        invert_steering = extractBool(content, "invert_steering");
        discrete_values = extractBool(content, "discrete_values");
        
        // RC values
        if ((val = extractInt(content, "min")) >= 0) rc_min = val;
        if ((val = extractInt(content, "neutral")) >= 0) rc_neutral = val;
        if ((val = extractInt(content, "max")) >= 0) rc_max = val;
        
        // Gear bits
        if ((val = extractInt(content, "gear_up")) >= 0) gear_up_bit = val;
        if ((val = extractInt(content, "gear_down")) >= 0) gear_down_bit = val;
    }
};

#endif
