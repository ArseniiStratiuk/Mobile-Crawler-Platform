#!/usr/bin/env python3
"""
AprilTag Follower - Entry Point
Tracks AprilTag and controls vehicle with combined steering + throttle.
"""

from config import (
    APRILTAG_FAMILY, TARGET_TAG_ID,
    TARGET_GEAR, STOP_AREA_RATIO,
    SERIAL_PORT, BAUD_RATE
)
from detection import AprilTagDetector
from follower import AprilTagFollower


def main():
    print("AprilTag Follower")
    print(f"Target: {APRILTAG_FAMILY} ID={TARGET_TAG_ID}")
    print(f"Gear: {TARGET_GEAR}, Proximity: {STOP_AREA_RATIO*100:.1f}%")
    print(f"Port: {SERIAL_PORT} @ {BAUD_RATE}")
    print()
    
    if not AprilTagDetector.is_available():
        print("ERROR: apriltag library not installed!")
        print("Install: pip install apriltag")
        return
    
    # Create and run follower
    follower = AprilTagFollower(SERIAL_PORT, BAUD_RATE)
    follower.run()


if __name__ == "__main__":
    main()
