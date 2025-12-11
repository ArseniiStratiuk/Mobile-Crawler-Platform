import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize Sensor
try:
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    print("VL53L0X Detected!")
except Exception as e:
    print(f"Error connecting to VL53L0X: {e}")
    exit()

print("Reading distance (Ctrl+C to stop)...")
try:
    while True:
        dist = sensor.range
        print(f"Distance: {dist} mm")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nExiting.")
