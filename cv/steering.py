#!/usr/bin/env python3
"""
Motion Controller - Discrete burst-based control for steering AND throttle.
Both channels use timed bursts: ON for duration, then OFF.
"""

import time
from config import (
    FRAME_WIDTH, CENTER_DEADZONE_PERCENT,
    STEER_LEFT, STEER_RIGHT, STEER_NEUTRAL,
    STEER_MIN_BURST, STEER_MAX_BURST, STEER_BURST_COOLDOWN,
    THROTTLE_NEUTRAL, THROTTLE_FORWARD,
    THROTTLE_MIN_BURST, THROTTLE_MAX_BURST, THROTTLE_BURST_COOLDOWN,
    STOP_AREA_RATIO
)
from pid_controller import PIDController


class MotionController:
    """
    Discrete burst-based control for both steering and throttle.
    
    How it works:
    - STEERING: Burst left/right for calculated duration, then neutral
    - THROTTLE: Burst forward for calculated duration, then neutral
    - Both can run simultaneously (different RC channels)
    - Burst duration calculated from PID output (steering) or distance (throttle)
    """
    
    def __init__(self, steer_kp, steer_ki, steer_kd):
        # Steering PID (calculates burst duration from centering error)
        self.steer_pid = PIDController(steer_kp, steer_ki, steer_kd)
        
        # Frame center
        self.center_x = FRAME_WIDTH / 2
        self.deadzone = FRAME_WIDTH * CENTER_DEADZONE_PERCENT
        
        # STEERING burst state
        self.steer_burst_active = False
        self.steer_burst_start = 0.0
        self.steer_burst_duration = 0.0
        self.steer_burst_direction = STEER_NEUTRAL
        self.last_steer_burst_end = 0.0
        
        # THROTTLE burst state  
        self.throttle_burst_active = False
        self.throttle_burst_start = 0.0
        self.throttle_burst_duration = 0.0
        self.last_throttle_burst_end = 0.0
        
        # Status
        self.status = "IDLE"
    
    def calculate_control(self, tag_center_x, tag_area_ratio, dt):
        """
        Calculate discrete control outputs for steering and throttle.
        
        Returns: (steer_pwm, throttle_pwm)
        Both are discrete: ON (left/right/forward) or OFF (neutral)
        """
        now = time.time()
        
        # Default outputs
        steer_pwm = STEER_NEUTRAL
        throttle_pwm = THROTTLE_NEUTRAL
        
        # Check proximity - if too close, turn-only mode
        in_proximity = tag_area_ratio >= STOP_AREA_RATIO
        
        # Calculate centering error
        error = tag_center_x - self.center_x
        centered = abs(error) < self.deadzone
        
        # --- STEERING BURST LOGIC ---
        steer_pwm = self._process_steering_burst(error, centered, dt, now)
        
        # --- THROTTLE BURST LOGIC ---
        if in_proximity:
            # Too close - no forward thrust, just turning allowed
            self.status = "PROXIMITY" if centered else "PROXIMITY+TURN"
            throttle_pwm = THROTTLE_NEUTRAL
        else:
            # Not too close - can move forward
            throttle_pwm = self._process_throttle_burst(centered, tag_area_ratio, now)
        
        return steer_pwm, throttle_pwm
    
    def _process_steering_burst(self, error, centered, dt, now):
        """Handle steering burst state machine."""
        
        if self.steer_burst_active:
            # Currently in a steering burst
            elapsed = now - self.steer_burst_start
            if elapsed >= self.steer_burst_duration:
                # Burst complete
                self.steer_burst_active = False
                self.last_steer_burst_end = now
                return STEER_NEUTRAL
            else:
                # Still bursting
                return self.steer_burst_direction
        
        # Not in burst - check if we should start one
        if centered:
            self.steer_pid.reset()
            self.status = "CENTERED"
            return STEER_NEUTRAL
        
        # Check cooldown
        if (now - self.last_steer_burst_end) < STEER_BURST_COOLDOWN:
            return STEER_NEUTRAL
        
        # Calculate burst from PID
        pid_output = abs(self.steer_pid.compute(error))
        burst_duration = min(STEER_MAX_BURST, max(STEER_MIN_BURST, pid_output * 0.1))
        
        # Start new steering burst
        self.steer_burst_active = True
        self.steer_burst_start = now
        self.steer_burst_duration = burst_duration
        self.steer_burst_direction = STEER_LEFT if error > 0 else STEER_RIGHT
        
        direction = "LEFT" if error > 0 else "RIGHT"
        self.status = f"STEER_{direction}"
        
        return self.steer_burst_direction
    
    def _process_throttle_burst(self, centered, tag_area_ratio, now):
        """
        Handle throttle burst state machine.
        
        Throttle burst duration based on:
        - Longer bursts when target is further away (smaller area ratio)
        - Shorter bursts when target is closer (larger area ratio)
        - Slightly shorter bursts when not centered (favor turning first)
        """
        
        if self.throttle_burst_active:
            # Currently in a throttle burst
            elapsed = now - self.throttle_burst_start
            if elapsed >= self.throttle_burst_duration:
                # Burst complete
                self.throttle_burst_active = False
                self.last_throttle_burst_end = now
                return THROTTLE_NEUTRAL
            else:
                # Still bursting
                return THROTTLE_FORWARD
        
        # Check cooldown
        if (now - self.last_throttle_burst_end) < THROTTLE_BURST_COOLDOWN:
            return THROTTLE_NEUTRAL
        
        # Calculate burst duration based on distance (inverse of area ratio)
        # Smaller area = further away = longer burst
        # Larger area = closer = shorter burst
        distance_factor = 1.0 - min(1.0, tag_area_ratio / STOP_AREA_RATIO)
        
        # If not centered, reduce forward intensity to favor turning
        centering_factor = 1.0 if centered else 0.6
        
        burst_duration = THROTTLE_MIN_BURST + (THROTTLE_MAX_BURST - THROTTLE_MIN_BURST) * distance_factor * centering_factor
        
        # Start new throttle burst
        self.throttle_burst_active = True
        self.throttle_burst_start = now
        self.throttle_burst_duration = burst_duration
        
        if "STEER" in self.status:
            self.status = self.status.replace("STEER", "FWD+STEER")
        elif centered:
            self.status = "FORWARD"
        
        return THROTTLE_FORWARD
    
    def stop(self):
        """Stop all motion."""
        self.steer_burst_active = False
        self.throttle_burst_active = False
        self.steer_pid.reset()
        self.status = "STOPPED"
        return STEER_NEUTRAL, THROTTLE_NEUTRAL
    
    def get_status(self):
        """Get current motion status for display."""
        return self.status
