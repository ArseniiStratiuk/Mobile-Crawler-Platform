#!/usr/bin/env python3
"""
AprilTag Follower - Tracks AprilTag 36h11 (ID=0) and steers to keep it centered.
Uses PID control with discrete steering bursts via MAVLink commands.
Gear 2 is used for consistent turn speed.
"""

import cv2
import numpy as np
import time
import subprocess
import threading
from pymavlink import mavutil

try:
    import apriltag
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("WARNING: apriltag library not installed!")
    print("Install with: pip install apriltag")

# Camera
CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# AprilTag
APRILTAG_FAMILY = "tag36h11"
TARGET_TAG_ID = 0
CENTER_DEADZONE_PERCENT = 0.10

# Video streaming
STREAM_HOST = '0.0.0.0'
STREAM_PORT = 12341

# MAVLink
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 57600

# RC Channels
STEER_CHANNEL = 1
MOVING_CHANNEL = 3
GEAR_CHANNEL = 6

# Steering PWM
STEER_LEFT = 1000
STEER_RIGHT = 2000
STEER_NEUTRAL = 1500

# Gear PWM
GEAR_NEUTRAL = 1500
GEAR_UP = 2000
GEAR_DOWN = 1000

# Throttle
NEUTRAL_MOVEMENT = 1500

SENDING_TIME = 0.51
NO_OVERRIDE = 65535

# PID gains
KP = 2.0
KI = 0.05
KD = 0.5

# Burst steering
MIN_BURST_DURATION = 0.1
MAX_BURST_DURATION = 0.5
BURST_COOLDOWN = 0.2


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.integral_limit = 100.0
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.01
        
        p_term = self.kp * error
        
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        self.last_time = current_time
        
        return p_term + i_term + d_term


class AprilTagFollower:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.master = None
        self.pid = PIDController(KP, KI, KD)
        
        self.running = True
        self.current_gear = None
        self.steer_pwm = STEER_NEUTRAL
        
        self.last_heartbeat_time = time.time()
        self.last_control_send_time = time.time()
        
        self.burst_active = False
        self.burst_start_time = 0
        self.burst_duration = 0
        self.last_burst_end_time = 0
        
        self.gear_command_active = False
        self.gear_command_channel = None
        self.gear_command_value = None
        self.gear_command_end_time = None
        self.target_gear = 2  # Gear 2 for consistent turn speed
        self.gear_set = False
        
        self.cap = None
        self.detector = None
        self.ffmpeg_process = None
        
        self.frame_center_x = FRAME_WIDTH // 2
        self.deadzone_pixels = int(FRAME_WIDTH * CENTER_DEADZONE_PERCENT)
        
        self.last_detection = None
        self.last_detection_time = 0
    
    def init_apriltag_detector(self):
        if not APRILTAG_AVAILABLE:
            print("ERROR: apriltag library not available!")
            return False
        
        print(f"Initializing AprilTag detector for family: {APRILTAG_FAMILY}")
        
        options = apriltag.DetectorOptions(
            families=APRILTAG_FAMILY,
            border=1,
            nthreads=4,
            quad_decimate=2.0,
            quad_blur=0.0,
            refine_edges=True,
            refine_decode=False,
            refine_pose=False,
            debug=False,
            quad_contours=True
        )
        
        self.detector = apriltag.Detector(options)
        print("AprilTag detector initialized!")
        return True
    
    def connect_mavlink(self):
        print(f"Connecting to {self.port} at {self.baud} baud...")
        self.master = mavutil.mavlink_connection(
            self.port, 
            baud=self.baud, 
            source_system=255, 
            mavlink2=True
        )
        
        print("Waiting for vehicle heartbeat...")
        self.master.wait_heartbeat()
        print(f"Vehicle connected! System: {self.master.target_system}, Component: {self.master.target_component}")
    
    def arm(self):
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
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.last_heartbeat_time = time.time()
    
    def check_messages(self):
        msg = self.master.recv_match(type='ESTIMATOR_STATUS', blocking=False)
        if msg and msg.vel_ratio != self.current_gear:
            print(f"GEAR UPDATE: {self.current_gear} -> {msg.vel_ratio}")
            self.current_gear = msg.vel_ratio
    
    def start_gear_change(self, channel, value, duration=SENDING_TIME):
        print(f"Starting gear change: Channel {channel}, PWM {value}, Duration {duration}s")
        self.gear_command_active = True
        self.gear_command_channel = channel
        self.gear_command_value = value
        self.gear_command_end_time = time.time() + duration
    
    def gear_up(self):
        print("Shifting GEAR UP")
        self.start_gear_change(GEAR_CHANNEL, GEAR_UP, SENDING_TIME)
    
    def gear_down(self):
        print("Shifting GEAR DOWN")
        self.start_gear_change(GEAR_CHANNEL, GEAR_DOWN, SENDING_TIME)
    
    def set_target_gear(self):
        if self.gear_set:
            return
        
        if self.current_gear is None:
            print(f"Gear unknown, shifting up to find gear...")
            self.gear_up()
        elif self.current_gear < self.target_gear:
            print(f"Current gear {self.current_gear}, shifting up to {self.target_gear}...")
            self.gear_up()
        elif self.current_gear > self.target_gear:
            print(f"Current gear {self.current_gear}, shifting down to {self.target_gear}...")
            self.gear_down()
        else:
            print(f"Already in target gear {self.target_gear}")
            self.gear_set = True
    
    def send_control_signals(self):
        overrides = [NO_OVERRIDE] * 18
        
        overrides[STEER_CHANNEL - 1] = self.steer_pwm
        overrides[MOVING_CHANNEL - 1] = NEUTRAL_MOVEMENT
        
        if self.gear_command_active:
            if time.time() < self.gear_command_end_time:
                overrides[self.gear_command_channel - 1] = self.gear_command_value
            else:
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
    
    def init_camera(self):
        print(f"Initializing camera {CAMERA_ID}...")
        self.cap = cv2.VideoCapture(CAMERA_ID)
        
        if not self.cap.isOpened():
            print(f"ERROR: Cannot open camera {CAMERA_ID}")
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera resolution: {self.actual_width}x{self.actual_height}")
        
        self.frame_center_x = self.actual_width // 2
        self.deadzone_pixels = int(self.actual_width * CENTER_DEADZONE_PERCENT)
        
        print(f"Center X: {self.frame_center_x}, Deadzone: {self.deadzone_pixels} pixels")
        return True
    
    def grab_latest_frame(self):
        self.cap.grab()
        ret, frame = self.cap.read()
        return ret, frame
    
    def init_video_stream(self):
        print(f"Starting video stream on port {STREAM_PORT}")
        
        self.stream_width = getattr(self, 'actual_width', FRAME_WIDTH)
        self.stream_height = getattr(self, 'actual_height', FRAME_HEIGHT)
        
        self.stream_ready = threading.Event()
        self.ffmpeg_process = None
        
        self.stream_thread = threading.Thread(target=self._start_ffmpeg, daemon=True)
        self.stream_thread.start()
        
        print(f"FFmpeg: {self.stream_width}x{self.stream_height}")
        print(f"Connect with: ffplay -fflags nobuffer tcp://<IP>:{STREAM_PORT}")
        return True
    
    def _start_ffmpeg(self):
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.stream_width}x{self.stream_height}',
            '-r', '30',
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-f', 'mpegts',
            f'tcp://0.0.0.0:{STREAM_PORT}?listen=1'
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdout=subprocess.DEVNULL
            )
            self.stream_ready.set()
        except Exception as e:
            print(f"FFmpeg start error: {e}")
    
    def stream_frame(self, frame):
        if self.ffmpeg_process is not None and self.ffmpeg_process.poll() is None:
            try:
                self.ffmpeg_process.stdin.write(frame.tobytes())
            except:
                pass
    
    def cleanup_stream(self):
        if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process is not None:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2)
            except:
                try:
                    self.ffmpeg_process.kill()
                except:
                    pass
            self.ffmpeg_process = None
    
    def detect_apriltag(self, frame):
        if self.detector is None:
            return False, None, None, None, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        
        for detection in detections:
            if detection.tag_id == TARGET_TAG_ID:
                center_x = detection.center[0]
                center_y = detection.center[1]
                corners = detection.corners
                
                width = np.linalg.norm(corners[0] - corners[1])
                height = np.linalg.norm(corners[1] - corners[2])
                tag_size = (width + height) / 2
                
                return True, center_x, center_y, corners, tag_size
        
        return False, None, None, None, None
    
    def calculate_steering(self, tag_center_x):
        error = tag_center_x - self.frame_center_x
        
        if abs(error) < self.deadzone_pixels:
            self.pid.reset()
            return False, 'none', 0
        
        pid_output = self.pid.compute(error)
        
        if pid_output > 0:
            direction = 'right'
        else:
            direction = 'left'
        
        normalized_output = abs(pid_output) / FRAME_WIDTH
        duration = MIN_BURST_DURATION + normalized_output * (MAX_BURST_DURATION - MIN_BURST_DURATION)
        duration = np.clip(duration, MIN_BURST_DURATION, MAX_BURST_DURATION)
        
        return True, direction, duration
    
    def check_burst_status(self):
        if not self.burst_active:
            return False
        
        current_time = time.time()
        elapsed = current_time - self.burst_start_time
        
        if elapsed >= self.burst_duration:
            self.steer_pwm = STEER_NEUTRAL
            self.burst_active = False
            self.last_burst_end_time = current_time
            print(f"Steering burst ENDED after {elapsed:.2f}s")
            return False
        
        return True
    
    def update_steering(self, should_steer, direction, duration):
        current_time = time.time()
        
        if self.burst_active:
            return
        
        if current_time - self.last_burst_end_time < BURST_COOLDOWN:
            return
        
        if should_steer and direction != 'none':
            if direction == 'left':
                self.steer_pwm = STEER_LEFT
                print(f"Starting LEFT burst for {duration:.2f}s")
            else:
                self.steer_pwm = STEER_RIGHT
                print(f"Starting RIGHT burst for {duration:.2f}s")
            
            self.burst_active = True
            self.burst_start_time = current_time
            self.burst_duration = duration
            
            self.send_control_signals()
        else:
            self.steer_pwm = STEER_NEUTRAL
    
    def draw_debug_info(self, frame, found, center_x=None, center_y=None, 
                        corners=None, tag_size=None, error=None):
        height, width = frame.shape[:2]
        
        cv2.line(frame, (self.frame_center_x, 0), (self.frame_center_x, height), (0, 255, 0), 2)
        
        left_boundary = self.frame_center_x - self.deadzone_pixels
        right_boundary = self.frame_center_x + self.deadzone_pixels
        cv2.line(frame, (left_boundary, 0), (left_boundary, height), (0, 255, 255), 1)
        cv2.line(frame, (right_boundary, 0), (right_boundary, height), (0, 255, 255), 1)
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (left_boundary, 0), (right_boundary, height), (0, 100, 0), -1)
        cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
        
        if found and corners is not None:
            corners_int = corners.astype(int)
            cv2.polylines(frame, [corners_int], True, (0, 255, 0), 2)
            
            if center_x is not None and center_y is not None:
                cx, cy = int(center_x), int(center_y)
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
                cv2.line(frame, (self.frame_center_x, cy), (cx, cy), (255, 0, 0), 2)
                
                if error is not None:
                    cv2.putText(frame, f"Error: {error:.1f}px", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if tag_size is not None:
                    cv2.putText(frame, f"Tag size: {tag_size:.1f}px", (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(frame, f"ID: {TARGET_TAG_ID}", 
                       (int(corners[0][0]), int(corners[0][1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        steer_text = "NEUTRAL"
        steer_color = (255, 255, 255)
        if self.steer_pwm == STEER_LEFT:
            steer_text = "LEFT"
            steer_color = (0, 255, 255)
        elif self.steer_pwm == STEER_RIGHT:
            steer_text = "RIGHT"
            steer_color = (0, 255, 255)
        
        cv2.putText(frame, f"Steering: {steer_text}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, steer_color, 2)
        
        gear_names = {0: "STOP", 1: "SLOW", 2: "NORMAL", 3: "FAST"}
        gear_text = f"Gear: {self.current_gear}"
        if self.current_gear in gear_names:
            gear_text += f" ({gear_names[self.current_gear]})"
        elif self.current_gear is None:
            gear_text = "Gear: Unknown"
        cv2.putText(frame, gear_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        status_text = "TRACKING" if found else "SEARCHING..."
        status_color = (0, 255, 0) if found else (0, 0, 255)
        cv2.putText(frame, status_text, (width - 150, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        cv2.putText(frame, "MODE: Turn-in-Place", (width - 200, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return frame
    
    def run(self):
        if not self.init_apriltag_detector():
            return
        
        if not self.init_camera():
            return
        
        self.init_video_stream()
        self.connect_mavlink()
        
        if not self.arm():
            self.cap.release()
            self.cleanup_stream()
            return
        
        print(f"\nAprilTag Follower Active")
        print(f"Target: {APRILTAG_FAMILY}, ID={TARGET_TAG_ID}")
        print(f"Mode: Turn-in-Place, Target Gear: {self.target_gear}")
        print(f"Video stream: tcp://<IP>:{STREAM_PORT}")
        print("Press Ctrl+C to quit\n")
        
        frame_count = 0
        
        try:
            while self.running:
                current_time = time.time()
                
                if current_time - self.last_heartbeat_time >= 1.0:
                    self.send_heartbeat()
                
                self.check_messages()
                
                if not self.gear_set and not self.gear_command_active:
                    self.set_target_gear()
                    if self.current_gear == self.target_gear:
                        self.gear_set = True
                
                ret, frame = self.grab_latest_frame()
                if not ret:
                    print("ERROR: Failed to read frame")
                    continue
                
                frame_count += 1
                
                found, center_x, center_y, corners, tag_size = self.detect_apriltag(frame)
                
                error = None
                if found:
                    error = center_x - self.frame_center_x
                    should_steer, direction, duration = self.calculate_steering(center_x)
                    self.update_steering(should_steer, direction, duration)
                    
                    if frame_count % 30 == 0:
                        in_deadzone = "IN DEADZONE" if abs(error) < self.deadzone_pixels else ""
                        print(f"Tag at ({center_x:.1f}, {center_y:.1f}), Error: {error:.1f}px, Size: {tag_size:.1f}px {in_deadzone}")
                else:
                    if not self.burst_active:
                        self.steer_pwm = STEER_NEUTRAL
                        self.pid.reset()
                    
                    if frame_count % 30 == 0:
                        print("AprilTag not detected, searching...")
                
                burst_is_active = self.check_burst_status()
                
                send_interval = 0.05 if burst_is_active else 0.1
                if current_time - self.last_control_send_time >= send_interval:
                    self.send_control_signals()
                    if burst_is_active:
                        elapsed = current_time - self.burst_start_time
                        print(f"[BURST] PWM={self.steer_pwm}, elapsed={elapsed:.2f}s/{self.burst_duration:.2f}s")
                
                debug_frame = self.draw_debug_info(frame, found, center_x, center_y, corners, tag_size, error)
                self.stream_frame(debug_frame)
        
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected. Stopping...")
        
        finally:
            self.steer_pwm = STEER_NEUTRAL
            self.send_control_signals()
            time.sleep(0.1)
            
            self.disarm()
            self.cap.release()
            self.cleanup_stream()
            print("AprilTag follower stopped.")


def main():
    print("AprilTag Follower - Turn-in-Place Mode")
    print(f"Tracks AprilTag {APRILTAG_FAMILY} (ID={TARGET_TAG_ID})")
    print()
    
    if not APRILTAG_AVAILABLE:
        print("ERROR: apriltag library not installed!")
        print("Install with: pip install apriltag")
        print("On some systems: pip install pupil-apriltags")
        return
    
    follower = AprilTagFollower(SERIAL_PORT, BAUD_RATE)
    follower.run()


if __name__ == "__main__":
    main()
