from smbus2 import SMBus
import time

# Addresses
ACCEL_ADDR = 0x19
GYRO_ADDR  = 0x69
MAG_ADDR   = 0x13

# Registers
MAG_POWER_REG = 0x4B  # Power control register
MAG_OP_MODE   = 0x4C  # Operation mode register
MAG_CHIP_ID   = 0x40  # Chip ID register

bus = SMBus(1)

def wakeup_mag():
    try:
        # 1. Soft Reset (optional, but good practice) - Write 0x82 to 0x4B
        bus.write_byte_data(MAG_ADDR, MAG_POWER_REG, 0x82)
        time.sleep(0.1)
        
        # 2. Power ON - Write 0x01 to 0x4B
        bus.write_byte_data(MAG_ADDR, MAG_POWER_REG, 0x01)
        time.sleep(0.1)
        
        # 3. Read ID
        val = bus.read_byte_data(MAG_ADDR, MAG_CHIP_ID)
        print(f"Magnetometer Wake-up attempt... Chip ID: 0x{val:02x}")
        
        if val == 0x32:
            print("SUCCESS: Magnetometer is awake and ready!")
        else:
            print("WARNING: Magnetometer ID is still unexpected.")
            
    except Exception as e:
        print(f"Mag Error: {e}")

print("--- Fixing Magnetometer ---")
wakeup_mag()
