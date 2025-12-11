#!/usr/bin/env python3
"""
AprilTag Follower - Main controller that ties all modules together.
Uses discrete burst control for both steering and throttle.
"""

import time

from config import (
    SERIAL_PORT, BAUD_RATE,
    STREAM_PORT,
    STEER_NEUTRAL, THROTTLE_NEUTRAL,
    TARGET_GEAR, GEAR_NAMES,
    STOP_AREA_RATIO,
    CONTROL_SEND_INTERVAL,
    APRILTAG_FAMILY, TARGET_TAG_ID,
    STEER_KP, STEER_KI, STEER_KD,
    TAG_LOSS_THROTTLE_GRACE
)
from camera import Camera, VideoStreamer
from detection import AprilTagDetector
from mavlink_controller import MAVLinkController
from steering import MotionController
from visualization import DebugVisualizer


class AprilTagFollower:
    """
    Main AprilTag follower controller.
    Coordinates all modules for discrete burst-based steering + throttle control.
    """
    
    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD_RATE):
        self.camera = Camera()
        self.streamer = None
        self.detector = AprilTagDetector()
        self.mavlink = MAVLinkController(port, baud)
        self.motion = MotionController(STEER_KP, STEER_KI, STEER_KD)
        self.visualizer = None
        
        self.running = True
        self.target_gear = TARGET_GEAR
        self.last_time = time.time()
        
        # Tag loss grace period state (throttle continues briefly)
        self.last_tag_seen_time = 0.0
        self.last_throttle_pwm = THROTTLE_NEUTRAL
        self.last_area_ratio = 0.0
        
        # Gear correction state
        self.gear_correction_attempts = 0
        self.max_gear_correction_attempts = 3
        self.last_gear_correction_time = 0
        self.gear_correction_cooldown = 2.0  # seconds between correction attempts
    
    def init_all(self) -> bool:
        """Initialize all modules."""
        if not AprilTagDetector.is_available():
            print("ERROR: apriltag library not available!")
            return False
        
        if not self.detector.init():
            return False
        
        if not self.camera.init():
            return False
        
        self.visualizer = DebugVisualizer(
            self.camera.frame_center_x,
            int(self.camera.actual_width * 0.10)  # 10% deadzone
        )
        
        self.streamer = VideoStreamer(
            self.camera.actual_width,
            self.camera.actual_height
        )
        self.streamer.start()
        
        if not self.mavlink.connect():
            return False
        
        return True
    
    def setup_vehicle(self) -> bool:
        """Arm vehicle and set gear."""
        if not self.mavlink.arm():
            return False
        
        print("\n--- Waiting for gear reading ---")
        if not self.mavlink.wait_for_gear_reading(timeout=10):
            print("Failed to get gear reading.")
            self.mavlink.disarm()
            return False
        
        print(f"\n--- Setting gear to {self.target_gear} ({GEAR_NAMES.get(self.target_gear, '?')}) ---")
        if not self.mavlink.set_gear_and_wait(self.target_gear, timeout=15):
            print("Failed to set gear.")
            self.mavlink.disarm()
            return False
        
        print(f"\n--- GEAR SET TO {int(self.mavlink.current_gear)} ---")
        return True
    
    def check_and_correct_gear(self) -> bool:
        """
        Check if gear is correct, attempt to fix if not.
        Returns True if gear is OK, False if we should skip this frame.
        """
        current = self.mavlink.current_gear
        if current == self.target_gear:
            self.gear_correction_attempts = 0
            return True
        
        now = time.time()
        
        # Too many failed attempts - give up
        if self.gear_correction_attempts >= self.max_gear_correction_attempts:
            print(f"GEAR ERROR: Failed to correct gear after {self.max_gear_correction_attempts} attempts!")
            print(f"Current: {current}, Target: {self.target_gear}")
            self.mavlink.stop()
            return False
        
        # Cooldown between attempts
        if (now - self.last_gear_correction_time) < self.gear_correction_cooldown:
            return False
        
        # Attempt gear correction
        self.gear_correction_attempts += 1
        self.last_gear_correction_time = now
        
        print(f"GEAR MISMATCH: {current} != {self.target_gear}, correcting (attempt {self.gear_correction_attempts})...")
        self.mavlink.stop()
        
        # Try to set correct gear
        if self.mavlink.set_gear_and_wait(self.target_gear, timeout=5):
            print(f"Gear corrected to {self.target_gear}")
            self.gear_correction_attempts = 0
            return True
        else:
            print(f"Gear correction failed, will retry...")
            return False
    
    def process_frame(self, frame, dt, frame_count):
        """
        Process a single frame: detect tag, calculate control, send commands.
        Returns the debug frame for streaming.
        """
        detection = self.detector.detect(frame)
        
        steer_pwm = STEER_NEUTRAL
        throttle_pwm = THROTTLE_NEUTRAL
        error = None
        in_proximity = False
        
        now = time.time()
        
        if detection.found:
            # Tag visible - calculate control normally
            steer_pwm, throttle_pwm = self.motion.calculate_control(
                detection.center_x,
                detection.area_ratio,
                dt
            )
            
            error = detection.center_x - self.motion.center_x
            in_proximity = detection.area_ratio >= STOP_AREA_RATIO
            
            # Remember for grace period
            self.last_tag_seen_time = now
            self.last_throttle_pwm = throttle_pwm
            self.last_area_ratio = detection.area_ratio
            
            if frame_count % 30 == 0:
                status = self.motion.get_status()
                print(f"Tag ({detection.center_x:.0f},{detection.center_y:.0f}) "
                      f"Err:{error:.0f}px Area:{detection.area_ratio*100:.2f}% "
                      f"S:{steer_pwm} T:{throttle_pwm} [{status}]")
        else:
            # Tag lost - stop steering immediately
            self.motion.steer_burst_active = False
            self.motion.steer_pid.reset()
            steer_pwm = STEER_NEUTRAL
            
            # Check grace period for throttle
            time_since_tag = now - self.last_tag_seen_time
            
            if time_since_tag < TAG_LOSS_THROTTLE_GRACE and self.last_area_ratio < STOP_AREA_RATIO:
                # Within grace period and wasn't in proximity - keep throttle going
                throttle_pwm = self.last_throttle_pwm
                self.motion.status = "TAG_LOST_COASTING"
                
                if frame_count % 30 == 0:
                    remaining = TAG_LOSS_THROTTLE_GRACE - time_since_tag
                    print(f"Tag lost, coasting... ({remaining:.1f}s remaining)")
            else:
                # Grace period expired or was in proximity - full stop
                self.motion.throttle_burst_active = False
                throttle_pwm = THROTTLE_NEUTRAL
                self.motion.status = "STOPPED"
                
                if frame_count % 30 == 0:
                    print("Tag lost, stopped.")
        
        # Send control signals
        self.mavlink.steer_pwm = steer_pwm
        self.mavlink.throttle_pwm = throttle_pwm
        self.mavlink.send_control_signals()
        
        # Draw debug visualization
        debug_frame = self.visualizer.draw(
            frame,
            found=detection.found,
            center_x=detection.center_x,
            center_y=detection.center_y,
            corners=detection.corners,
            tag_area=detection.tag_area,
            area_ratio=detection.area_ratio,
            error=error,
            steer_pwm=steer_pwm,
            throttle_pwm=throttle_pwm,
            current_gear=self.mavlink.current_gear,
            in_proximity=in_proximity
        )
        
        return debug_frame
    
    def run(self):
        """Main control loop."""
        if not self.init_all():
            return
        
        if not self.setup_vehicle():
            self.cleanup()
            return
        
        print(f"\nTracking: {APRILTAG_FAMILY} ID={TARGET_TAG_ID}")
        print(f"Gear: {TARGET_GEAR} ({GEAR_NAMES.get(TARGET_GEAR, '?')})")
        print(f"Proximity threshold: {STOP_AREA_RATIO*100:.1f}% of frame")
        print(f"Stream: tcp://<IP>:{STREAM_PORT}")
        print("Ctrl+C to quit\n")
        
        frame_count = 0
        self.last_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                
                # MAVLink housekeeping
                self.mavlink.check_heartbeat_needed()
                self.mavlink.check_messages()
                
                # Gear watchdog - auto-correct if wrong
                if not self.check_and_correct_gear():
                    time.sleep(0.1)
                    continue
                
                # Grab frame
                ret, frame = self.camera.grab_frame()
                if not ret:
                    continue
                
                frame_count += 1
                
                # Process and stream
                debug_frame = self.process_frame(frame, dt, frame_count)
                self.streamer.send_frame(debug_frame)
        
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources."""
        self.mavlink.stop()
        time.sleep(0.1)
        self.mavlink.disarm()
        self.camera.release()
        if self.streamer:
            self.streamer.stop()
        print("Stopped.")
