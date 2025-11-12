import time
import sys
import threading
from pymavlink import mavutil

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 57600

# RC Channels (from mentor's information)
THROTTLE_CHANNEL = 6  # Channel 6 forward-backward
STEER_CHANNEL = 1     # Channel 1 left-right

# PWM values (from "default 1500")
NEUTRAL_PWM = 1500
FORWARD_PWM = 2000
BACKWARD_PWM = 1300
LEFT_PWM = 1300
RIGHT_PWM = 1700

# Gear buttons (from mentor's information)
GEAR_UP_BIT = 2048    # bit 11
GEAR_DOWN_BIT = 4096  # bit 12

class RoverController:
    def __init__(self, port, baud):
        """Establishes connection with MAVLink."""
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

        # Current control state
        self.throttle_pwm = NEUTRAL_PWM
        self.steer_pwm = NEUTRAL_PWM

        self.current_gear = None
        self.message_listener_thread = threading.Thread(target=self._message_listener_loop, daemon=True)
        
        self.running = True
        self.override_thread = threading.Thread(target=self._rc_override_loop, daemon=True)
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
    
    def _message_listener_loop(self):
        """
        (NEW THREAD)
        This thread listens to incoming MAVLink messages in the background.
        It searches for ESTIMATOR_STATUS to get vel_ratio.
        """
        print("Starting message listener thread...")
        msg = 0
        while self.running:
            # We wait (blocking=True) for ESTIMATOR_STATUS message.
            # If the message doesn't arrive within 1.0 seconds (timeout),
            # the loop simply repeats. This is efficient and doesn't load the CPU.
            msg = self.master.recv_match(
                type='ESTIMATOR_STATUS', 
                blocking=True, 
                timeout=1.0
            )
            
            if not msg:
                # Timeout, message didn't arrive, just continue the loop
                continue

            if msg.vel_ratio != self.current_gear:
                print(f"\nGEAR UPDATE: {self.current_gear} -> {msg.vel_ratio}")
                self.current_gear = msg.vel_ratio
        print(msg)


    def _rc_override_loop(self):
        """
        This function runs in a background thread.
        It CONTINUOUSLY sends joystick state (RC_CHANNELS_OVERRIDE).
        This is critical, otherwise the robot will activate failsafe.
        """
        while self.running:
            # Create an array of 18 channels filled with NEUTRAL_PWM
            overrides = [NEUTRAL_PWM] * 18
            
            # Set steering and throttle values
            overrides[STEER_CHANNEL - 1] = self.steer_pwm
            overrides[THROTTLE_CHANNEL - 1] = self.throttle_pwm
            
            # Send RC_CHANNELS_OVERRIDE message (70)
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *overrides
            )
            time.sleep(0.1) # Send 10 times per second

    def _heartbeat_loop(self):
        """
        This function runs in a background thread.
        It CONTINUOUSLY sends HEARTBEAT message once per second.
        This is necessary to maintain connection with the autopilot.
        """
        while self.running:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,  # base_mode
                0,  # custom_mode
                0   # system_status
            )
            time.sleep(1.0)  # Send once per second

    def send_pwm_pulse(self, channel_number, pwm_value, duration_sec=0.7):
        """
        (NEW FUNCTION)
        Sends a PWM pulse to the specified channel for a certain time.
        Other channels during the pulse will be at NEUTRAL_PWM (1500).
        """
        print(f"Sending PWM pulse: {pwm_value} on Channel {channel_number} for {duration_sec}s")
        
        # Create an array with NEUTRAL_PWM (1500)
        overrides = [NEUTRAL_PWM] * 18
        
        # Set target PWM value (remember zero-based indexing)
        overrides[channel_number - 1] = pwm_value
        
        # Array for returning to neutral state (all 1500)
        neutral_overrides = [NEUTRAL_PWM] * 18

        start_time = time.time()
        send_interval = 0.05  # Send 20 times per second
        last_send_time = 0
        
        while time.time() - start_time < duration_sec:
            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *overrides
                )
                last_send_time = current_time
            time.sleep(0.01) # Prevent 100% CPU load

        # Return all channels to NEUTRAL
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *neutral_overrides
        )
        print(f"Pulse complete. Channel {channel_number} reset to {NEUTRAL_PWM}.")

    def _send_button_press(self, button_bitmask):
        """Sends ONE MANUAL_CONTROL message (69) to simulate button press."""
        print(f"Sending button press: {button_bitmask}")
        # Press button
        self.master.mav.manual_control_send(
            self.master.target_system,
            0, # x (pitch)
            0, # y (roll)
            0, # z (throttle)
            0, # r (yaw)
            button_bitmask # buttons
        )
        # Wait 0.1s
        time.sleep(0.1)
        # Release button (send 0)
        self.master.mav.manual_control_send(
            self.master.target_system, 0, 0, 0, 0, 0
        )
        print("Button released")

    def arm(self):
        """Arms the robot."""
        print("Attempting to ARM...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # param1: 1 to arm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for acknowledgment
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
            0,  # confirmation
            0,  # param1: 0 to disarm
            0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"DISARMED. ACK: {ack}")

    def gear_up(self):
        print("Shifting GEAR UP")
        self.send_pwm_pulse(channel_number=6, pwm_value=2000, duration_sec=0.7)

    def gear_down(self):
        print("Shifting GEAR DOWN")
        #not working
        if self.current_gear == 0.0:
            # --- NEW LOGIC FOR REVERSE ---
            print("Current gear is 0. Engaging REVERSE (Channel 3)...")
            # Send signal to Channel 3, PWM 1000
            self.send_pwm_pulse(channel_number=3, pwm_value=1000, duration_sec=0.7)
            # Expect the tank to report vel_ratio -1.0

        elif self.current_gear > 0.0:
            # --- OLD LOGIC (normal downshift) ---
            print(f"Current gear {self.current_gear}. Downshifting (Channel 6)...")
            # Send signal to Channel 6, PWM 1000
            self.send_pwm_pulse(channel_number=6, pwm_value=1000, duration_sec=0.7)
        
        else:
            # This means we're already in reverse (e.g., -1.0)
            print(f"Already in reverse mode or impossible gear ({self.current_gear}). No action.")

    def process_command(self, cmd):
        """Processes user-entered commands."""
        if cmd == 'w':
            print("Moving FORWARD")
            self.throttle_pwm = FORWARD_PWM
        elif cmd == 's':
            print("Moving BACKWARD")
            self.throttle_pwm = BACKWARD_PWM
        elif cmd == 'a':
            print("Turning LEFT")
            self.steer_pwm = LEFT_PWM
        elif cmd == 'd':
            print("Turning RIGHT")
            self.steer_pwm = RIGHT_PWM
        elif cmd == 'stop':
            print("STOPPING")
            self.throttle_pwm = NEUTRAL_PWM
            self.steer_pwm = NEUTRAL_PWM
        elif cmd == 'e':
            print("GEAR UP")
            self.gear_up()
        elif cmd == 'q':
            print("GEAR DOWN")
            self.gear_down()
        elif cmd == 'gear':
            print(f"Current reported gear: {self.current_gear}")
        elif cmd == 'test':
            print("TEST: Sending gear up signal")
            self.send_pwm_pulse(channel_number=6, pwm_value=2000, duration_sec=0.7)
        elif cmd == 'quit':
            print("Quitting...")
            self.running = False
        else:
            print("Unknown command")

    def start(self):
        """Main loop that waits for commands."""
        if not self.arm():
            self.running = False
            return
            
        # Start background thread for RC_CHANNELS_OVERRIDE
        self.override_thread.start()
        
        # Start background thread for HEARTBEAT
        self.heartbeat_thread.start()

        # Start background thread for message listening
        self.message_listener_thread.start()
        
        print("\n--- Rover Control Ready ---")
        print(" w = Forward")
        print(" s = Backward")
        print(" a = Left")
        print(" d = Right")
        print(" e = Gear Up")
        print(" q = Gear Down")
        print(" gear = Show current gear")
        print(" test = Test gear signal (Ch6: 2000->1500)")
        print(" stop = STOP")
        print(" quit = Disarm and Exit")
        print(" (Ctrl+D = Disarm and Exit)")
        print("---------------------------")
        
        try:
            while self.running:
                # Wait for user instruction
                cmd = input("Enter command: ").strip().lower()
                self.process_command(cmd)
                
        except EOFError:
            print("\nEOFError (Ctrl+D) detected. Quitting...")
            self.running = False
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt (Ctrl+C) detected. Quitting...")
            self.running = False

    def stop(self):
        """Stops all processes and disarms the robot."""
        print("Stopping controller...")
        self.running = False
        if self.override_thread.is_alive():
            self.override_thread.join()
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join()
        if self.message_listener_thread.is_alive():
            self.message_listener_thread.join()
        
        # Set neutral values before disarming
        self.throttle_pwm = NEUTRAL_PWM
        self.steer_pwm = NEUTRAL_PWM
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *([NEUTRAL_PWM] * 18)
        )
        time.sleep(0.5)
        self.disarm()
        print("Controller stopped. Robot disarmed.")

# --- Main launch block ---
if __name__ == "__main__":
    controller = RoverController(SERIAL_PORT, BAUD_RATE)
    try:
        controller.start()
    finally:
        # This 'finally' block ensures that the robot will be disarmed
        # even if the program crashes with an error.
        controller.stop()
