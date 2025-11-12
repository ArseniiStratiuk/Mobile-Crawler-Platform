# Joystick Configuration

The joystick handler now supports runtime configuration via JSON file, allowing you to change all mappings and parameters without recompiling.

## Configuration File

Default location: `config.json` in the same directory as the executable.

### Complete Configuration Example

```json
{
  "joystick": {
    "throttle_axis": 1,
    "steering_axis": 3,
    "gear_up_button": 0,
    "gear_down_button": 1,
    "arm_button": 7,
    "disarm_button": 6
  },
  "control": {
    "deadzone": 0.5,
    "invert_throttle": true,
    "invert_steering": false,
    "discrete_values": true
  },
  "rc_values": {
    "min": 1000,
    "neutral": 1500,
    "max": 2000
  },
  "gear_bits": {
    "gear_up": 11,
    "gear_down": 12
  }
}
```

## Configuration Parameters

### Joystick Section
- **`throttle_axis`**: SDL axis number for throttle control (default: 1 = left stick Y)
- **`steering_axis`**: SDL axis number for steering control (default: 3 = right stick X)
- **`gear_up_button`**: Button number for gear up (default: 0 = A button)
- **`gear_down_button`**: Button number for gear down (default: 1 = B button)
- **`arm_button`**: Button to arm the vehicle (default: 7 = START)
- **`disarm_button`**: Button to disarm the vehicle (default: 6 = BACK/SELECT)

### Control Section
- **`deadzone`**: Deadzone threshold (0.0-1.0). Stick must exceed this to register (default: 0.5 = 50%)
- **`invert_throttle`**: Invert throttle axis direction (default: true)
- **`invert_steering`**: Invert steering axis direction (default: false)
- **`discrete_values`**: Use only min/neutral/max values (true) or continuous range (false) (default: true)

### RC Values Section
- **`min`**: Minimum RC value (default: 1000)
- **`neutral`**: Neutral/center RC value (default: 1500)
- **`max`**: Maximum RC value (default: 2000)

### Gear Bits Section
- **`gear_up`**: Bit position for gear up in button bitmask (default: 11)
- **`gear_down`**: Bit position for gear down in button bitmask (default: 12)

## Xbox 360 Controller Mapping Reference

### Axes
```
0 = Left Stick X
1 = Left Stick Y
2 = Left Trigger
3 = Right Stick X
4 = Right Stick Y
5 = Right Trigger
```

### Buttons
```
0 = A (bottom, green)
1 = B (right, red)
2 = X (left, blue)
3 = Y (top, yellow)
4 = LB (left bumper)
5 = RB (right bumper)
6 = BACK (select)
7 = START
8 = Left Stick Click
9 = Right Stick Click
```

## Usage

### Command Line
```bash
# Use default config.json in current directory
./test_joystick

# Specify custom config file
./test_joystick /path/to/my_config.json
```

### Example Configurations

#### Racing Setup (Triggers for throttle/brake)
```json
{
  "joystick": {
    "throttle_axis": 5,
    "steering_axis": 0
  },
  "control": {
    "deadzone": 0.1,
    "invert_throttle": false,
    "discrete_values": false
  }
}
```

#### Simple 3-Position Control
```json
{
  "control": {
    "deadzone": 0.5,
    "discrete_values": true
  }
}
```

## Testing Your Configuration

1. Copy `config.json` to your build directory
2. Edit the values as needed
3. Run `./test_joystick`
4. The program will print loaded configuration
5. Test joystick movements and verify behavior

No recompilation needed! Just edit the JSON and restart the program.
