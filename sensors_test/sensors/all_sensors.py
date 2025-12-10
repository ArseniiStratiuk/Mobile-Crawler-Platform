"""Simple sensor hub - reads all sensors and displays them"""

import time
import threading
from . import distance, imu


class SensorHub:
    """
    Reads all sensors and provides easy access to data.
    Can run in background thread for continuous updates.
    """
    
    def __init__(self):
        self.data = {
            'distance_mm': None,
            'heading': 0.0,
            'accel': (0.0, 0.0, 0.0),
            'gyro': (0.0, 0.0, 0.0),
            'mag': (0.0, 0.0, 0.0),
            'pitch': 0.0,
            'roll': 0.0,
            'timestamp': 0
        }
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
    
    def read_once(self):
        """Read all sensors once and update data"""
        with self._lock:
            self.data['distance_mm'] = distance.get_distance()
            self.data['heading'] = imu.get_heading()
            self.data['accel'] = imu.get_accel()
            self.data['gyro'] = imu.get_gyro()
            self.data['mag'] = imu.get_mag()
            pitch, roll = imu.get_tilt()
            self.data['pitch'] = pitch
            self.data['roll'] = roll
            self.data['timestamp'] = time.time()
        
        return self.get_data()
    
    def get_data(self):
        """Get current sensor data (thread-safe)"""
        with self._lock:
            return self.data.copy()
    
    def start(self, rate_hz=20):
        """Start continuous sensor reading in background thread"""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, args=(rate_hz,), daemon=True)
        self._thread.start()
        print(f"[SensorHub] Started reading at {rate_hz} Hz")
    
    def stop(self):
        """Stop background reading"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        print("[SensorHub] Stopped")
    
    def _read_loop(self, rate_hz):
        """Background reading loop"""
        interval = 1.0 / rate_hz
        
        while self._running:
            try:
                self.read_once()
                time.sleep(interval)
            except Exception as e:
                print(f"[SensorHub] Error: {e}")
                time.sleep(0.5)
    
    def print_data(self):
        """Print current sensor data in a nice format"""
        data = self.get_data()
        
        print("\n" + "=" * 60)
        print("                    SENSOR DATA")
        print("=" * 60)
        
        # Distance
        dist = data['distance_mm']
        if dist:
            dist_m = dist / 1000.0
            print(f"Distance:    {dist} mm ({dist_m:.2f} m)")
        else:
            print(f"Distance:    No reading")
        
        print()
        
        # Orientation
        heading = data['heading']
        if heading is not None:
            print(f"Heading:     {heading:6.1f}°  {_heading_direction(heading)}")
        else:
            print(f"Heading:     N/A (Magnetometer not initialized)")
        print(f"Pitch:       {data['pitch']:6.1f}°")
        print(f"Roll:        {data['roll']:6.1f}°")
        
        print()
        
        # Acceleration
        ax, ay, az = data['accel']
        print(f"Accel X:     {ax:7.2f} m/s²")
        print(f"Accel Y:     {ay:7.2f} m/s²")
        print(f"Accel Z:     {az:7.2f} m/s²")
        
        print()
        
        # Gyro
        gx, gy, gz = data['gyro']
        print(f"Gyro X:      {gx:7.1f} °/s")
        print(f"Gyro Y:      {gy:7.1f} °/s")
        print(f"Gyro Z:      {gz:7.1f} °/s")
        
        print("=" * 60)


def _heading_direction(heading):
    """Convert heading to compass direction"""
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    idx = int((heading + 22.5) / 45) % 8
    return directions[idx]


if __name__ == "__main__":
    hub = SensorHub()
    
    print("Reading sensors continuously...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            hub.read_once()
            hub.print_data()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped")
