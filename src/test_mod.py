import time
from pymavlink import mavutil

class MAVLinkTester:
    def __init__(self, port='/dev/ttyS0', baud=57600):
        self.m = mavutil.mavlink_connection(port, baud=baud, source_system=255, dialect='ardupilotmega', mavlink2=True)
        self.m.port.flushInput()
        self.send_gcs_heartbeat()
        print("Waiting for first heartbeat...")
        self.m.wait_heartbeat(timeout=5)
        print("Connected.")

    def send_gcs_heartbeat(self):
        self.m.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=0
        )

    def send_command(self, command, params=(0,0,0,0,0,0,0), target_component=1):
        self.m.mav.command_long_send(
            target_system=1, target_component=target_component, command=command,
            confirmation=0, param1=params[0], param2=params[1], param3=params[2], param4=params[3],
            param5=params[4], param6=params[5], param7=params[6]
        )
        ack = self.m.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        return ack.result if ack else None

    def monitor(self, duration=1, types=['VFR_HUD', 'SYS_STATUS', 'HEARTBEAT']):
        print(f"Monitoring for {duration}s (focus: {', '.join(types)})...")
        start = time.time()
        while time.time() - start < duration:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg and msg.get_type() in types:
                print(f"{msg.get_type()}: {msg}")

    def set_rc_override(self, channel, pwm, duration=5):
        print(f"Overriding channel {channel} to {pwm} for {duration}s...")
        overrides = [0] * 8  # Only 8 channels supported
        if 1 <= channel <= 8:
            overrides[channel - 1] = pwm
        else:
            print("Invalid channel (1-8 only).")
            return
        self.m.mav.rc_channels_override_send(1, 1, *overrides)
        time.sleep(duration)
        self.m.mav.rc_channels_override_send(1, 1, *([0]*8))  # Clear
        print("Override cleared.")

if __name__ == "__main__":
    tester = MAVLinkTester()
    
    print("\nBefore any changes:")
    tester.monitor(duration=1)
    
    print("\nDisarming first (if armed, base_mode to 64)...")
    tester.send_command(400, (0, 0, 0, 0, 0, 0, 0))
    
    print("\nArming (watch HEARTBEAT base_mode to 192)...")
    arm_result = tester.send_command(400, (1, 0, 0, 0, 0, 0, 0))
    print(f"Arm result: {arm_result} (0=success)")
    
    print("\nAfter arm:")
    tester.monitor(duration=1)
    
    print("\nSetting mode to MANUAL (for RC override, custom_mode=0)...")
    mode_result = tester.send_command(176, (1, 0, 0, 0, 0, 0, 0))
    print(f"Mode result: {mode_result}")
    
    print("\nTesting throttle override (channel 3 to 1600, watch VFR_HUD throttle >0)...")
    tester.set_rc_override(3, 1600, duration=5)
    
    print("\nDuring/after throttle:")
    tester.monitor(duration=1)
    
    print("\nDisarming (base_mode back to 64)...")
    tester.send_command(400, (0, 0, 0, 0, 0, 0, 0))
    
    print("\nFinal status:")
    tester.monitor(duration=1)
