from smbus2 import SMBus
import time

# I2C Addresses from your i2cdetect
ACCEL_ADDR = 0x19
GYRO_ADDR  = 0x69
MAG_ADDR   = 0x13

# Register Map (Who Am I / Chip ID)
ACCEL_CHIP_ID_REG = 0x00
GYRO_CHIP_ID_REG  = 0x00
MAG_CHIP_ID_REG   = 0x40

bus = SMBus(1)

def read_id(addr, reg, name):
    try:
        val = bus.read_byte_data(addr, reg)
        print(f"{name} (0x{addr:02x}): CONNECTED - Chip ID: 0x{val:02x}")
        return True
    except Exception as e:
        print(f"{name} (0x{addr:02x}): FAILED - {e}")
        return False

print("--- BMX055 Health Check ---")
read_id(ACCEL_ADDR, ACCEL_CHIP_ID_REG, "Accelerometer") # Expected: 0xFA
read_id(GYRO_ADDR, GYRO_CHIP_ID_REG, "Gyroscope    ") # Expected: 0x0F
read_id(MAG_ADDR, MAG_CHIP_ID_REG, "Magnetometer ") # Expected: 0x32
