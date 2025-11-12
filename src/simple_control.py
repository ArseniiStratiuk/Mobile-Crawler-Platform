import time
from pymavlink import mavutil

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 57600

# RC Channels
STEER_CHANNEL = 1       # Steering channel
REVERSE_CHANNEL = 3     # Reverse channel
GEAR_CHANNEL = 6        # Gear channel

# PWM values for steering
STEER_LEFT = 1000
STEER_RIGHT = 2000
STEER_NEUTRAL = 1500

# PWM values for gears
GEAR_NEUTRAL = 1500
GEAR_UP = 2000
GEAR_DOWN = 1000

MOVING_CHANNEL = 3
FORWARD_MOVEMENT = 2000
REVERSE_MOVEMENT = 1000
NEUTRAL_MOVEMENT = 1500

SENDING_TIME = 0.51

NO_OVERRIDE = 65535

class SimpleRoverController:
    def __init__(self, port, baud):
        print(f"Connecting to {port} at {baud} baud...")
        self.master = mavutil.mavlink_connection(
            port, 
            baud=baud, 
            source_system=255, 
            mavlink2=True
        )
        
        # Wait for first HEARTBEAT
        print("Waiting for vehicle heartbeat...")
        self.master.wait_heartbeat()
        print("Vehicle connected!")
        
        # Current state
        self.current_gear = None
        self.running = True
        self.steer_pwm = STEER_NEUTRAL  # Current steering value
        self.drive_or_reverse = NEUTRAL_MOVEMENT
        
        # Timers for periodic tasks
        self.last_heartbeat_time = time.time()
        self.last_message_check_time = time.time()
        self.last_control_send_time = time.time()
        
        # Active gear command state
        self.gear_command_active = False
        self.gear_command_channel = None
        self.gear_command_value = None
        self.gear_command_end_time = None

    def arm(self):
        """Arms the robot."""
        print("Attempting to ARM...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            print("ARMED successfully!")
            return True
        else:
            print(f"Failed to ARM! ACK: {ack}")
            return False

    def disarm(self):
        """Disarms the robot."""
        print("Attempting to DISARM...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"DISARMED. ACK: {ack}")

    def send_heartbeat(self):
        """Sends HEARTBEAT message."""
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.last_heartbeat_time = time.time()

    def check_messages(self):
        """Checks incoming MAVLink messages."""
        msg = self.master.recv_match(type='ESTIMATOR_STATUS', blocking=False)
        if msg and msg.vel_ratio != self.current_gear:
            print(f"\nGEAR UPDATE: {self.current_gear} -> {msg.vel_ratio}")
            self.current_gear = msg.vel_ratio
        self.last_message_check_time = time.time()

    def send_control_signals(self):
        """Sends a single control command for all channels."""
        overrides = [NO_OVERRIDE] * 18

        # Steering
        overrides[STEER_CHANNEL - 1] = self.steer_pwm
        
        # Movement
        overrides[MOVING_CHANNEL - 1] = self.drive_or_reverse

        # Gears
        if self.gear_command_active:
            if time.time() < self.gear_command_end_time:
                overrides[self.gear_command_channel - 1] = self.gear_command_value
            else:
                # Time is up, send neutral and finish
                overrides[self.gear_command_channel - 1] = GEAR_NEUTRAL
                print(f"Gear command complete. Channel {self.gear_command_channel} reset to {GEAR_NEUTRAL}")
                self.gear_command_active = False
                self.gear_command_channel = None
                self.gear_command_value = None
                self.gear_command_end_time = None
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *overrides
        )
        self.last_control_send_time = time.time()

    def start_gear_change(self, channel, value, duration=SENDING_TIME):
        """Start gear change."""
        print(f"Starting gear change: Channel {channel}, PWM {value}, Duration {duration}s")
        self.gear_command_active = True
        self.gear_command_channel = channel
        self.gear_command_value = value
        self.gear_command_end_time = time.time() + duration

    def gear_up(self):
        """Shift gear up."""
        print("Shifting GEAR UP")
        self.start_gear_change(GEAR_CHANNEL, GEAR_UP, SENDING_TIME)

    def gear_down(self):
        """Shift gear down."""
        print("Shifting GEAR DOWN")
        
        if self.current_gear is None:
            print("Current gear unknown. Sending gear down signal...")
            self.start_gear_change(GEAR_CHANNEL, GEAR_DOWN, SENDING_TIME)
        else:
            print(f"Current gear {self.current_gear}. Lowering gear on Channel 6...")
            self.start_gear_change(GEAR_CHANNEL, GEAR_DOWN, SENDING_TIME)

    def process_command(self, cmd):
        """Processes input commands."""
        if cmd == 'e':
            self.gear_up()
        elif cmd == 'q':
            self.gear_down()
        elif cmd == 'a':
            print("Turning LEFT")
            self.steer_pwm = STEER_LEFT
        elif cmd == 'd':
            print("Turning RIGHT")
            self.steer_pwm = STEER_RIGHT
        elif cmd == 's':
            print("Steering NEUTRAL")
            self.steer_pwm = STEER_NEUTRAL
        elif cmd == 'gear':
            print(f"Current reported gear: {self.current_gear}")
        elif cmd == 'quit':
            print("Quitting...")
            self.running = False
        elif cmd == 'w':
            self.drive_or_reverse = FORWARD_MOVEMENT
            print("Moving forward...")
        elif cmd == 'r':
            self.drive_or_reverse = REVERSE_MOVEMENT
            print("Moving reverse...")
        elif cmd == 'x':
            self.drive_or_reverse = NEUTRAL_MOVEMENT
            print("Neutral moving...")
        else:
            print("Unknown command")

    def run(self):
        """Main control loop."""
        if not self.arm():
            self.running = False
            return
        
        print("\n--- Rover Control Ready ---")
        print(" e = Gear Up")
        print(" q = Gear Down")
        print(" a = Steer Left")
        print(" d = Steer Right")
        print(" s = Steer Neutral")
        print(" gear = Show current gear")
        print(" quit = Disarm and Exit")
        print(" w = move forward")
        print(" r = move backward")
        print(" x = stop moving")
        print(f" Target: sys={self.master.target_system}, comp={self.master.target_component}")
        print("---------------------------\n")
        
        try:
            while self.running:
                current_time = time.time()
                
                # 1. Send heartbeat once per second
                if current_time - self.last_heartbeat_time >= 1.0:
                    self.send_heartbeat()
                
                # 2. Check incoming messages every 0.1 seconds
                if current_time - self.last_message_check_time >= 0.1:
                    self.check_messages()
                
                # 3. Send combined control signals every 0.1 seconds
                if current_time - self.last_control_send_time >= 0.1:
                    self.send_control_signals()
                
                # 4. Check user input (non-blocking)
                import select
                import sys
                if select.select([sys.stdin], [], [], 0)[0]:
                    cmd = sys.stdin.readline().strip().lower()
                    if cmd:
                        self.process_command(cmd)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected. Quitting...")
            self.running = False
        
        self.disarm()
        print("Controller stopped. Robot disarmed.")

# --- Main launch block ---
if __name__ == "__main__":
    controller = SimpleRoverController(SERIAL_PORT, BAUD_RATE)
    controller.run()
