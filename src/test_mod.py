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

    def press_button_and_watch(self, channel, pwm_active, pwm_default, duration):
        """
        Sends a specific PWM value on one channel while holding all others at default.
        Monitors ALL incoming MAVLink messages for ANY change.
        """
        print(f"\n--- PRESSING BUTTON (Ch{channel}={pwm_active}) for {duration}s ---")
        print("--- Monitoring ALL messages for ANY field changes... ---")
        
        # Set up the override array with all channels at default
        overrides = [pwm_default] * 18
        # Set the specific channel to its "active" value
        overrides[channel - 1] = pwm_active

        start_time = time.time()
        send_interval = 0.05  # Send 20 times per second (50ms)
        last_send_time = 0
        
        # This dictionary will store the last known state of every message type
        previous_messages = {}
        
        # These fields always change and are not interesting (noise)
        ignore_fields = ['time_boot_ms', 'time_usec', 'airspeed', 'time_unix_usec', 'uptime', 'seq']
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # 1. Send the override command *continuously*
            if current_time - last_send_time > send_interval:
                self.m.mav.rc_channels_override_send(1, 1, *overrides)
                last_send_time = current_time

            # 2. Listen for ANY message
            msg = self.m.recv_match(blocking=False)
            if msg and msg.get_type() != 'BAD_DATA':
                msg_type = msg.get_type()
                
                if msg_type not in previous_messages:
                    # First time seeing this message type
                    print(f"[FIRST SIGHTING] {msg_type}: {msg}")
                    previous_messages[msg_type] = msg
                else:
                    # We have seen this message before, let's compare
                    old_msg_dict = previous_messages[msg_type].to_dict()
                    new_msg_dict = msg.to_dict()
                    
                    for key, new_val in new_msg_dict.items():
                        if key in ignore_fields or key == 'mavpackettype':
                            continue
                        
                        old_val = old_msg_dict.get(key)
                        
                        # Use float-safe comparison if needed
                        is_different = False
                        if isinstance(new_val, float):
                            if abs(new_val - old_val) > 0.001:
                                is_different = True
                        elif old_val != new_val:
                            is_different = True

                        if is_different:
                            print(f"+++ CHANGE DETECTED [{msg_type}] -> {key}: FROM {old_val} TO {new_val} +++")
                            
                    # Update the snapshot
                    previous_messages[msg_type] = msg

            time.sleep(0.01)  # Prevent 100% CPU usage

        # 3. After the loop, send a command to clear all overrides back to default
        self.m.mav.rc_channels_override_send(1, 1, *([pwm_default]*18))
        print(f"--- Button Released (Ch{channel}={pwm_default}) ---")


# --- Main execution block ---
if __name__ == "__main__":
    try:
        tester = MAVLinkTester()
        
        DEFAULT_PWM = 1500
        ACTIVE_PWM = 1000
        BUTTON_CHANNEL = 6
        
        print("\nAttempting to ARM (command 400, param1=1)...")
        arm_result = tester.send_command(400, (1, 0, 0, 0, 0, 0, 0))
        
        if arm_result == 0: # 0 = MAV_RESULT_ACCEPTED
            print("\nVehicle is ARMED. Beginning button press sequence.")
            
            # Send neutral signal for 1 second to establish baseline
            print("\n--- Sending NEUTRAL (1500) on all channels for 1s ---")
            tester.m.mav.rc_channels_override_send(1, 1, *([DEFAULT_PWM]*18))
            time.sleep(1)

            # --- 1. LONG PRESS A ---
            tester.press_button_and_watch(
                channel=BUTTON_CHANNEL, 
                pwm_active=ACTIVE_PWM, 
                pwm_default=DEFAULT_PWM, 
                duration=3  # "Long press" for 3 seconds
            )
            
            # Wait for 1 second in neutral state
            print("\n--- Holding NEUTRAL for 1s ---")
            tester.m.mav.rc_channels_override_send(1, 1, *([DEFAULT_PWM]*18))
            time.sleep(1)

            # --- 2. SHORT PRESS A ---
            tester.press_button_and_watch(
                channel=BUTTON_CHANNEL, 
                pwm_active=ACTIVE_PWM, 
                pwm_default=DEFAULT_PWM, 
                duration=0.5 # "Short press" for 0.5 seconds
            )
            
            # Wait for 1 second in neutral state
            print("\n--- Holding NEUTRAL for 1s ---")
            tester.m.mav.rc_channels_override_send(1, 1, *([DEFAULT_PWM]*18))
            time.sleep(1)

            print("\nButton press sequence complete.")
            
        else:
            print(f"\nDid not arm (Result: {arm_result}). Skipping button test.")

        print("\nAttempting to DISARM (command 400, param1=0)...")
        tester.send_command(400, (0, 0, 0, 0, 0, 0, 0))
        
        print("\nFinal status check:")
        tester.monitor(duration=1, types=['HEARTBEAT'])

    except Exception as e:
        print(f"An error occurred: {e}")
        print("Please check your serial port permissions and connection.")
