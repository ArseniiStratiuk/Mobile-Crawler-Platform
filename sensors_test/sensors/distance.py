"""VL53L0X Distance Sensor - Simple wrapper"""

import time

try:
    import board
    import busio
    import adafruit_vl53l0x
    HAS_SENSOR = True
except ImportError:
    HAS_SENSOR = False

_sensor = None
_last_distance = None


def _init():
    """Initialize sensor on first use"""
    global _sensor
    if _sensor is not None:
        return True
    
    if not HAS_SENSOR:
        print("[Distance] Libraries not available")
        return False
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        _sensor = adafruit_vl53l0x.VL53L0X(i2c)
        print("[Distance] VL53L0X initialized")
        return True
    except Exception as e:
        print(f"[Distance] Init failed: {e}")
        return False


def get_distance():
    """
    Get distance to obstacle in millimeters.
    Returns None if sensor not available or read fails.
    
    Returns:
        int: Distance in mm, or None
    """
    global _last_distance
    
    if not _init():
        return _last_distance
    
    try:
        dist = _sensor.range
        # Sanity check
        if 20 < dist < 8000:
            _last_distance = dist
            return dist
        return None
    except Exception:
        return _last_distance


def get_distance_meters():
    """
    Get distance in meters.
    Returns float('inf') if no obstacle detected.
    """
    mm = get_distance()
    if mm is None:
        return float('inf')
    return mm / 1000.0


if __name__ == "__main__":
    print("Testing distance sensor...")
    while True:
        dist = get_distance()
        if dist:
            print(f"Distance: {dist} mm ({dist/10:.1f} cm)")
        else:
            print("Distance: No reading")
        time.sleep(0.2)
