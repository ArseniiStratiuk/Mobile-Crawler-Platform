#!/usr/bin/env python3
"""
Visualization module for debug overlays.
"""

import cv2
import numpy as np
from typing import Optional

from config import (
    STEER_LEFT, STEER_RIGHT, STEER_NEUTRAL,
    THROTTLE_NEUTRAL, THROTTLE_FORWARD,
    TARGET_TAG_ID, GEAR_NAMES
)


class DebugVisualizer:
    """Draws debug information on video frames."""
    
    def __init__(self, frame_center_x: int, deadzone_pixels: int):
        self.frame_center_x = frame_center_x
        self.deadzone_pixels = deadzone_pixels
    
    def update_dimensions(self, frame_center_x: int, deadzone_pixels: int):
        self.frame_center_x = frame_center_x
        self.deadzone_pixels = deadzone_pixels
    
    def draw(self, frame: np.ndarray,
             found: bool,
             center_x: Optional[float] = None,
             center_y: Optional[float] = None,
             corners: Optional[np.ndarray] = None,
             tag_area: Optional[float] = None,
             area_ratio: Optional[float] = None,
             error: Optional[float] = None,
             steer_pwm: int = STEER_NEUTRAL,
             throttle_pwm: int = THROTTLE_NEUTRAL,
             current_gear: Optional[int] = None,
             in_proximity: bool = False) -> np.ndarray:
        """Draw debug info on frame."""
        height, width = frame.shape[:2]
        
        # Center line
        cv2.line(frame, (self.frame_center_x, 0), 
                (self.frame_center_x, height), (0, 255, 0), 2)
        
        # Deadzone boundaries
        left_boundary = self.frame_center_x - self.deadzone_pixels
        right_boundary = self.frame_center_x + self.deadzone_pixels
        cv2.line(frame, (left_boundary, 0), (left_boundary, height), (0, 255, 255), 1)
        cv2.line(frame, (right_boundary, 0), (right_boundary, height), (0, 255, 255), 1)
        
        # Deadzone overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (left_boundary, 0), (right_boundary, height), (0, 100, 0), -1)
        cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
        
        # Tag detection
        if found and corners is not None:
            corners_int = corners.astype(int)
            cv2.polylines(frame, [corners_int], True, (0, 255, 0), 2)
            
            if center_x is not None and center_y is not None:
                cx, cy = int(center_x), int(center_y)
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
                cv2.line(frame, (self.frame_center_x, cy), (cx, cy), (255, 0, 0), 2)
                
                if error is not None:
                    cv2.putText(frame, f"Err: {error:.0f}px", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if area_ratio is not None:
                    cv2.putText(frame, f"Ratio: {area_ratio*100:.2f}%", (10, 150), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(frame, f"ID:{TARGET_TAG_ID}", 
                       (int(corners[0][0]), int(corners[0][1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Steering status
        steer_text = "NEUTRAL"
        steer_color = (255, 255, 255)
        if steer_pwm == STEER_LEFT:
            steer_text = "LEFT"
            steer_color = (0, 255, 255)
        elif steer_pwm == STEER_RIGHT:
            steer_text = "RIGHT"
            steer_color = (0, 255, 255)
        cv2.putText(frame, f"Steer: {steer_text}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, steer_color, 2)
        
        # Throttle status (discrete: ON or OFF)
        if throttle_pwm == THROTTLE_FORWARD:
            thr_text = "FWD"
            thr_color = (0, 255, 0)
        else:
            thr_text = "OFF"
            thr_color = (200, 200, 200)
        cv2.putText(frame, f"Thr: {thr_text}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, thr_color, 2)
        
        # Gear status
        gear_text = f"Gear: {current_gear}"
        if current_gear in GEAR_NAMES:
            gear_text += f" ({GEAR_NAMES[current_gear]})"
        elif current_gear is None:
            gear_text = "Gear: ?"
        cv2.putText(frame, gear_text, (10, 130), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Status
        if in_proximity:
            status_text = "PROXIMITY (turn only)"
            status_color = (0, 165, 255)  # Orange
        elif found:
            status_text = "TRACKING"
            status_color = (0, 255, 0)
        else:
            status_text = "SEARCHING..."
            status_color = (0, 0, 255)
        cv2.putText(frame, status_text, (width - 200, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        return frame
