import serial
import time

# Set up serial connection based on your logs
# Note: NEO-6M default baud is usually 9600
ser = serial.Serial('/dev/ttyAMA3', baudrate=9600, timeout=1)

print("Listening for GPS Data... (Press Ctrl+C to stop)")

try:
    while True:
        # Read a line from the serial port
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Filter for GPGGA (Global Positioning System Fix Data)
            if line.startswith("$GPGGA"):
                parts = line.split(',')
                # Check if we have a fix (Quality > 0)
                if parts[6] != '0':
                    print(f"FIX FOUND! Lat: {parts[2]} {parts[3]}, Lon: {parts[4]} {parts[5]}")
                else:
                    print("Searching for Satellites... (No Fix)")
            
            # Optional: Print raw data to ensure stream is working
            # print(line) 
            
except KeyboardInterrupt:
    print("\nExiting.")
finally:
    ser.close()
