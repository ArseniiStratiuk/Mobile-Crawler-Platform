#!/usr/bin/env python3
"""
MAVLink Controller module.
"""

import time
from pymavlink import mavutil

from config import (
    SERIAL_PORT, BAUD_RATE,
    STEER_CHANNEL, MOVING_CHANNEL, GEAR_CHANNEL,
    STEER_NEUTRAL, THROTTLE_NEUTRAL,
    GEAR_NEUTRAL, GEAR_UP, GEAR_DOWN,
    SENDING_TIME, NO_OVERRIDE,
    HEARTBEAT_INTERVAL
)


class MAVLinkController:
    """MAVLink vehicle communication handler."""
    
    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD_RATE):
        self.port = port
        self.baud = baud
        self.master = None
        
        self.last_heartbeat_time = time.time()
        self.last_control_send_time = time.time()
        
        self.current_gear = None
        self.steer_pwm = STEER_NEUTRAL
        self.throttle_pwm = THROTTLE_NEUTRAL
        
        self.gear_command_active = False
        self.gear_command_channel = None
        self.gear_command_value = None
        self.gear_command_end_time = None
    
    def connect(self) -> bool:
        print(f"Connecting to {self.port} @ {self.baud}...")
        try:
            self.master = mavutil.mavlink_connection(
                self.port, baud=self.baud, 
                source_system=255, mavlink2=True
            )
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print(f"Connected: Sys={self.master.target_system}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def arm(self) -> bool:
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            print("Armed")
            return True
        print(f"Arm failed: {ack}")
        return False
    
    def disarm(self):
        print("Disarming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print("Disarmed")
    
    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.last_heartbeat_time = time.time()
    
    def check_heartbeat_needed(self) -> bool:
        if time.time() - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
            self.send_heartbeat()
            return True
        return False
    
    def check_messages(self) -> bool:
        msg = self.master.recv_match(type='ESTIMATOR_STATUS', blocking=False)
        if msg:
            new_gear = msg.vel_ratio
            if new_gear != self.current_gear:
                print(f"Gear: {self.current_gear} -> {new_gear}")
                self.current_gear = new_gear
            return True
        return False
    
    def wait_for_gear_reading(self, timeout: float = 10) -> bool:
        print("Waiting for gear reading...")
        start_time = time.time()
        
        while self.current_gear is None:
            if time.time() - start_time > timeout:
                print("Timeout waiting for gear!")
                return False
            self.send_heartbeat()
            self.check_messages()
            time.sleep(0.1)
        
        print(f"Got gear: {self.current_gear}")
        return True
    
    def set_gear_and_wait(self, target: int, timeout: float = 10) -> bool:
        print(f"Setting gear to {target} (current: {self.current_gear})")
        
        while self.current_gear != target:
            if self.current_gear < target:
                print(f"Shifting UP: {self.current_gear} -> {target}")
                self.gear_command_active = True
                self.gear_command_channel = GEAR_CHANNEL
                self.gear_command_value = GEAR_UP
                self.gear_command_end_time = time.time() + SENDING_TIME
            elif self.current_gear > target:
                print(f"Shifting DOWN: {self.current_gear} -> {target}")
                self.gear_command_active = True
                self.gear_command_channel = GEAR_CHANNEL
                self.gear_command_value = GEAR_DOWN
                self.gear_command_end_time = time.time() + SENDING_TIME
            
            start_time = time.time()
            while self.gear_command_active:
                if time.time() - start_time > timeout:
                    print("Timeout during gear change!")
                    return False
                self.send_heartbeat()
                self.send_control_signals()
                self.check_messages()
                time.sleep(0.05)
            
            wait_start = time.time()
            old_gear = self.current_gear
            while self.current_gear == old_gear:
                if time.time() - wait_start > 3:
                    print("Gear didn't change, retrying...")
                    break
                self.send_heartbeat()
                self.check_messages()
                time.sleep(0.1)
        
        print(f"Gear set to {self.current_gear}")
        return True
    
    def send_control_signals(self):
        """Send RC channel overrides."""
        overrides = [NO_OVERRIDE] * 18
        overrides[STEER_CHANNEL - 1] = self.steer_pwm
        overrides[MOVING_CHANNEL - 1] = self.throttle_pwm
        
        if self.gear_command_active:
            if time.time() < self.gear_command_end_time:
                overrides[self.gear_command_channel - 1] = self.gear_command_value
            else:
                overrides[self.gear_command_channel - 1] = GEAR_NEUTRAL
                print(f"Gear cmd done, ch{self.gear_command_channel}={GEAR_NEUTRAL}")
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
    
    def stop(self):
        """Set all controls to neutral."""
        self.steer_pwm = STEER_NEUTRAL
        self.throttle_pwm = THROTTLE_NEUTRAL
        self.send_control_signals()
