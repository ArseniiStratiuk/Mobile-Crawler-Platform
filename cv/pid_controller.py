#!/usr/bin/env python3
"""
PID Controller module.
"""

import time
import numpy as np

from config import STEER_KP, STEER_KI, STEER_KD, PID_INTEGRAL_LIMIT


class PIDController:
    """PID controller with anti-windup."""
    
    def __init__(self, kp: float = STEER_KP, ki: float = STEER_KI, kd: float = STEER_KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.integral_limit = PID_INTEGRAL_LIMIT
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, error: float) -> float:
        """Compute PID output for given error."""
        current_time = time.time()
        dt = max(current_time - self.last_time, 0.01)
        
        p_term = self.kp * error
        
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        self.last_time = current_time
        
        return p_term + i_term + d_term
    
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
