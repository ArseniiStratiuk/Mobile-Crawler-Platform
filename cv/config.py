#!/usr/bin/env python3
"""
Configuration settings for AprilTag Follower system.
All constants and parameters in one place for easy tuning.
"""

# Camera
CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_BUFFER_SIZE = 1

# AprilTag
APRILTAG_FAMILY = "tag36h11"
TARGET_TAG_ID = 0
CENTER_DEADZONE_PERCENT = 0.10  # 10% of frame width

# Detector options
APRILTAG_BORDER = 1
APRILTAG_NTHREADS = 4
APRILTAG_QUAD_DECIMATE = 2.0
APRILTAG_QUAD_BLUR = 0.0
APRILTAG_REFINE_EDGES = True
APRILTAG_REFINE_DECODE = False
APRILTAG_REFINE_POSE = False
APRILTAG_DEBUG = False
APRILTAG_QUAD_CONTOURS = True

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

# Steering PWM (discrete: LEFT, RIGHT, or NEUTRAL)
STEER_LEFT = 1000
STEER_RIGHT = 2000
STEER_NEUTRAL = 1500

# Gear PWM
GEAR_NEUTRAL = 1500
GEAR_UP = 2000
GEAR_DOWN = 1000

# Throttle PWM (discrete: FORWARD or NEUTRAL)
THROTTLE_NEUTRAL = 1500
THROTTLE_FORWARD = 2000

# Gear target
TARGET_GEAR = 2

# Gear name mapping
GEAR_NAMES = {
    0: "STOP",
    1: "SLOW",
    2: "NORMAL",
    3: "FAST"
}

# Proximity threshold - below this distance, turn-only mode (no forward)
STOP_AREA_RATIO = 0.005  # 0.5% of frame

# Timing
SENDING_TIME = 0.51
NO_OVERRIDE = 65535
HEARTBEAT_INTERVAL = 1.0

# Steering PID gains (for burst duration calculation)
STEER_KP = 2.0
STEER_KI = 0.05
STEER_KD = 0.5
PID_INTEGRAL_LIMIT = 100.0

# STEERING burst parameters
STEER_MIN_BURST = 0.05
STEER_MAX_BURST = 0.35
STEER_BURST_COOLDOWN = 0.5

# THROTTLE burst parameters (discrete forward pulses)
THROTTLE_MIN_BURST = 0.08      # Minimum forward burst
THROTTLE_MAX_BURST = 0.25      # Maximum forward burst  
THROTTLE_BURST_COOLDOWN = 0.05 # Short cooldown - smoother forward motion

# Tag loss grace period (throttle continues briefly when tag lost due to vibration)
TAG_LOSS_THROTTLE_GRACE = 0.8  # seconds to keep throttle going after tag loss

# Control loop timing
CONTROL_SEND_INTERVAL = 0.05
