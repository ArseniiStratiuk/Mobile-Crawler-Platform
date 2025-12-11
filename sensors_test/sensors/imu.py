"""BMX055 IMU - Simple wrapper for accelerometer, gyroscope, magnetometer"""

import math
import time

try:
    from smbus2 import SMBus
    HAS_SENSOR = True
except ImportError:
    HAS_SENSOR = False

# I2C Addresses
ACCEL_ADDR = 0x19
GYRO_ADDR = 0x69
MAG_ADDR = 0x13

# Registers
ACCEL_X_LSB = 0x02
GYRO_X_LSB = 0x02
MAG_X_LSB = 0x42
MAG_POWER_REG = 0x4B
MAG_OP_MODE = 0x4C
MAG_DATA_RATE = 0x4C
MAG_CHIP_ID = 0x40

_bus = None
_mag_initialized = False
_last_accel = (0.0, 0.0, 0.0)
_last_gyro = (0.0, 0.0, 0.0)
_last_mag = (0.0, 0.0, 0.0)
_last_heading = 0.0


def _init():
    """Initialize IMU on first use"""
    global _bus, _mag_initialized
    
    if _bus is not None:
        return True
    
    if not HAS_SENSOR:
        print("[IMU] smbus2 not available")
        return False
    
    try:
        _bus = SMBus(1)
        
        # Wake up magnetometer (sometimes needs multiple attempts)
        for attempt in range(3):
            try:
                # Soft reset
                _bus.write_byte_data(MAG_ADDR, MAG_POWER_REG, 0x82)
                time.sleep(0.15)  # Wait longer for reset
                
                # Power on
                _bus.write_byte_data(MAG_ADDR, MAG_POWER_REG, 0x01)
                time.sleep(0.15)  # Wait for power-up
                
                # Set operation mode: Normal mode, 10Hz data rate
                # 0x00 = Normal mode, 10Hz
                _bus.write_byte_data(MAG_ADDR, MAG_OP_MODE, 0x00)
                time.sleep(0.05)
                
                # Verify chip ID
                chip_id = _bus.read_byte_data(MAG_ADDR, MAG_CHIP_ID)
                if chip_id == 0x32:
                    _mag_initialized = True
                    print(f"[IMU] Magnetometer initialized (Chip ID: 0x32, attempt {attempt+1})")
                    break
                else:
                    print(f"[IMU] Magnetometer unexpected ID: 0x{chip_id:02x} (expected 0x32)")
                    if attempt < 2:
                        time.sleep(0.2)
                        
            except Exception as e:
                if attempt < 2:
                    print(f"[IMU] Magnetometer init attempt {attempt+1} failed: {e}, retrying...")
                    time.sleep(0.2)
                else:
                    print(f"[IMU] Magnetometer init failed after 3 attempts: {e}")
                    print("[IMU] Heading data will not be available")
        
        print("[IMU] BMX055 initialized")
        return True
        
    except Exception as e:
        print(f"[IMU] Init failed: {e}")
        return False


def _read_word_signed(addr, reg):
    """Read signed 16-bit value from I2C"""
    if not _init():
        return 0
    try:
        low = _bus.read_byte_data(addr, reg)
        high = _bus.read_byte_data(addr, reg + 1)
        val = (high << 8) | low
        if val >= 32768:
            val -= 65536
        return val
    except Exception:
        return 0


def get_accel():
    """
    Get accelerometer readings in m/s².
    Returns (x, y, z) tuple.
    
    Note: Can be used to estimate speed by integrating, but will drift!
    For speed, GPS is more accurate. Accel is good for detecting bumps, tilt.
    """
    global _last_accel
    
    if not _init():
        return _last_accel
    
    try:
        # BMX055 accel is 12-bit, ±2g range
        scale = 2.0 * 9.81 / 2048.0
        
        x = _read_word_signed(ACCEL_ADDR, ACCEL_X_LSB) >> 4
        y = _read_word_signed(ACCEL_ADDR, ACCEL_X_LSB + 2) >> 4
        z = _read_word_signed(ACCEL_ADDR, ACCEL_X_LSB + 4) >> 4
        
        _last_accel = (x * scale, y * scale, z * scale)
        return _last_accel
        
    except Exception:
        return _last_accel


def get_gyro():
    """
    Get gyroscope readings in degrees/second.
    Returns (x, y, z) tuple.
    
    Measures rotation rate. Can integrate to get angle, but drifts over time.
    """
    global _last_gyro
    
    if not _init():
        return _last_gyro
    
    try:
        # BMX055 gyro is 16-bit, ±2000°/s range
        scale = 2000.0 / 32768.0
        
        x = _read_word_signed(GYRO_ADDR, GYRO_X_LSB)
        y = _read_word_signed(GYRO_ADDR, GYRO_X_LSB + 2)
        z = _read_word_signed(GYRO_ADDR, GYRO_X_LSB + 4)
        
        _last_gyro = (x * scale, y * scale, z * scale)
        return _last_gyro
        
    except Exception:
        return _last_gyro


def get_mag():
    """
    Get magnetometer readings in arbitrary units.
    Returns (x, y, z) tuple.
    
    Use get_heading() for compass direction instead of raw values.
    """
    global _last_mag
    
    if not _init() or not _mag_initialized:
        return _last_mag
    
    try:
        # BMX055 mag is 13-bit for X/Y, 15-bit for Z
        x = _read_word_signed(MAG_ADDR, MAG_X_LSB) >> 3
        y = _read_word_signed(MAG_ADDR, MAG_X_LSB + 2) >> 3
        z = _read_word_signed(MAG_ADDR, MAG_X_LSB + 4) >> 1
        
        # Check if we got valid data
        if x != 0 or y != 0 or z != 0:
            _last_mag = (float(x), float(y), float(z))
        
        return _last_mag
        
    except Exception:
        return _last_mag


def get_heading():
    """
    Get compass heading in degrees (0-360).
    0° = North, 90° = East, 180° = South, 270° = West
    
    Returns None if magnetometer not available.
    
    Note: This assumes the sensor is level (horizontal).
    For better accuracy on slopes, need tilt compensation using accelerometer.
    """
    global _last_heading
    
    mag = get_mag()
    
    # Check if magnetometer is working
    if not _mag_initialized or mag == (0.0, 0.0, 0.0):
        return None  # Return None instead of last_heading to show it's not working
    
    mx, my, mz = mag
    
    # Calculate heading (assumes level sensor)
    heading = math.atan2(my, mx) * 180.0 / math.pi
    
    # Normalize to 0-360
    if heading < 0:
        heading += 360
    
    _last_heading = heading
    return heading


def get_tilt():
    """
    Get pitch and roll angles in degrees from accelerometer.
    Returns (pitch, roll) tuple.
    
    Pitch: nose up (+) or down (-)
    Roll: bank right (+) or left (-)
    """
    ax, ay, az = get_accel()
    
    # Calculate angles
    pitch = math.atan2(ay, math.sqrt(ax*ax + az*az)) * 180.0 / math.pi
    roll = math.atan2(-ax, az) * 180.0 / math.pi
    
    return (pitch, roll)


if __name__ == "__main__":
    print("Testing IMU...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            accel = get_accel()
            gyro = get_gyro()
            heading = get_heading()
            pitch, roll = get_tilt()
            
            print(f"Accel: X={accel[0]:6.2f} Y={accel[1]:6.2f} Z={accel[2]:6.2f} m/s²")
            print(f"Gyro:  X={gyro[0]:6.1f} Y={gyro[1]:6.1f} Z={gyro[2]:6.1f} °/s")
            print(f"Heading: {heading:6.1f}° | Pitch: {pitch:5.1f}° | Roll: {roll:5.1f}°")
            print("-" * 60)
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopped")
