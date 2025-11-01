import time
from pymavlink import mavutil

class MAVLinkTester:
    def __init__(self, port='/dev/ttyS0', baud=57600):
        # Connect to the MAVLink port
        self.m = mavutil.mavlink_connection(port, baud=baud, source_system=255, dialect='ardupilotmega', mavlink2=True)
        self.m.port.flushInput()
        
        # Send a GCS heartbeat to register this script as a ground station
        self.send_gcs_heartbeat()
        print("Waiting for first heartbeat from vehicle...")
        
        # Wait for the first heartbeat message to confirm connection
        self.m.wait_heartbeat(timeout=5)
        print("Vehicle connected.")

    def send_gcs_heartbeat(self):
        # Send a GCS heartbeat to identify this script to the vehicle
        self.m.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=0
        )

    def send_command(self, command, params=(0,0,0,0,0,0,0), target_component=1):
        # Send a MAV_CMD_COMMAND_LONG message
        self.m.mav.command_long_send(
            target_system=1, target_component=target_component, command=command,
            confirmation=0, param1=params[0], param2=params[1], param3=params[2], param4=params[3],
            param5=params[4], param6=params[5], param7=params[6]
        )
        
        # Wait for the COMMAND_ACK response
        ack = self.m.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        
        if ack:
            print(f"ACK for command {command}: {ack}")
            return ack.result
        else:
            print(f"No ACK received for command {command}")
            return None

    def monitor(self, duration=1, types=['VFR_HUD', 'SYS_STATUS', 'HEARTBEAT']):
        # Print out specified messages for a short duration
        print(f"Monitoring for {duration}s (focus: {', '.join(types)})...")
        start = time.time()
        while time.time() - start < duration:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg and msg.get_type() in types:
                print(f"{msg.get_type()}: {msg}")

    def test_continuous_throttle(self, channel, pwm, duration=5):
        """
        Sends RC_CHANNELS_OVERRIDE repeatedly to simulate a joystick.
        This is the correct way to test throttle.
        """
        print(f"\n--- TESTING CONTINUOUS THROTTLE (Channel {channel}={pwm} for {duration}s) ---")
        print("Looking for VFR_HUD.throttle > 0...")
        
        # Set up the override array. Channel 3 is throttle (index 2).
        overrides = [0] * 8  # 8 channels, all 0
        if 1 <= channel <= 8:
            # NOTE: Try all channels to make sure for the throttle test
            overrides[0] = pwm # Channel 1
            overrides[1] = pwm # Channel 2
            overrides[2] = pwm # Channel 3
            overrides[3] = pwm # Channel 4
        else:
            print("Invalid channel (1-8 only).")
            return

        start_time = time.time()
        send_interval = 0.1 # Send 10 times per second (1.0 / 10)
        last_send_time = 0
        success = False

        # Loop for the specified duration
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # 1. Send the override command *continuously*
            if current_time - last_send_time > send_interval:
                self.m.mav.rc_channels_override_send(1, 1, *overrides)
                last_send_time = current_time

            # 2. Listen for VFR_HUD messages at the same time
            msg = self.m.recv_match(type='VFR_HUD', blocking=False)
            if msg:
                # Print any VFR_HUD message we receive
                print(f"VFR_HUD: {msg}")
                # Check if the throttle field is non-zero
                if msg.throttle > 0:
                    success = True
            
            time.sleep(0.01) # Small sleep to prevent 100% CPU usage

        # 3. After the loop, send a command to clear all overrides
        self.m.mav.rc_channels_override_send(1, 1, *([0]*8))
        print("--- Continuous override test finished. Clearing channels. ---")
        
        # 4. Print the final result
        if success:
            print(">>> SUCCESS: Throttle value was observed > 0. <<<")
        else:
            print(">>> FAILURE: Throttle value remained 0. <<<")
        
        return success

# --- Main execution block ---
if __name__ == "__main__":
    try:
        tester = MAVLinkTester()
        
        print("\nAttempting to ARM (command 400, param1=1)...")
        # Send MAV_CMD_COMPONENT_ARM_DISARM (400) with param1=1 (arm)
        arm_result = tester.send_command(400, (1, 0, 0, 0, 0, 0, 0))
        
        if arm_result == 0: # 0 = MAV_RESULT_ACCEPTED
            print("\nVehicle is ARMED. Now testing continuous throttle...")
            
            # Test throttle (channel 3) at 1600 PWM for 10 seconds
            # This is the test for "giving it forward"
            tester.test_continuous_throttle(channel=3, pwm=1600, duration=10)
            
        else:
            print(f"\nDid not arm (Result: {arm_result}). Skipping throttle test.")

        print("\nAttempting to DISARM (command 400, param1=0)...")
        # Send MAV_CMD_COMPONENT_ARM_DISARM (400) with param1=0 (disarm)
        tester.send_command(400, (0, 0, 0, 0, 0, 0, 0))
        
        print("\nFinal status check:")
        tester.monitor(duration=1)

    except Exception as e:
        print(f"An error occurred: {e}")
        print("Please check your serial port permissions and connection.")
