#!/usr/bin/env python3
"""
AprilTag detection module.
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional

from config import (
    APRILTAG_FAMILY, TARGET_TAG_ID,
    APRILTAG_BORDER, APRILTAG_NTHREADS,
    APRILTAG_QUAD_DECIMATE, APRILTAG_QUAD_BLUR,
    APRILTAG_REFINE_EDGES, APRILTAG_REFINE_DECODE,
    APRILTAG_REFINE_POSE, APRILTAG_DEBUG,
    APRILTAG_QUAD_CONTOURS
)

try:
    import apriltag
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("WARNING: apriltag not installed (pip install apriltag)")


@dataclass
class TagDetection:
    """AprilTag detection result."""
    found: bool
    center_x: Optional[float] = None
    center_y: Optional[float] = None
    corners: Optional[np.ndarray] = None
    tag_area: Optional[float] = None
    area_ratio: Optional[float] = None
    tag_id: Optional[int] = None


class AprilTagDetector:
    """AprilTag detector."""
    
    def __init__(self, family: str = APRILTAG_FAMILY, target_id: int = TARGET_TAG_ID):
        self.family = family
        self.target_id = target_id
        self.detector = None
    
    @staticmethod
    def is_available() -> bool:
        return APRILTAG_AVAILABLE
    
    def init(self) -> bool:
        if not APRILTAG_AVAILABLE:
            print("ERROR: apriltag library not available!")
            return False
        
        print(f"Initializing AprilTag detector: {self.family}")
        
        options = apriltag.DetectorOptions(
            families=self.family,
            border=APRILTAG_BORDER,
            nthreads=APRILTAG_NTHREADS,
            quad_decimate=APRILTAG_QUAD_DECIMATE,
            quad_blur=APRILTAG_QUAD_BLUR,
            refine_edges=APRILTAG_REFINE_EDGES,
            refine_decode=APRILTAG_REFINE_DECODE,
            refine_pose=APRILTAG_REFINE_POSE,
            debug=APRILTAG_DEBUG,
            quad_contours=APRILTAG_QUAD_CONTOURS
        )
        
        self.detector = apriltag.Detector(options)
        print("Detector ready")
        return True
    
    def detect(self, frame: np.ndarray) -> TagDetection:
        """Detect target AprilTag in frame."""
        if self.detector is None:
            return TagDetection(found=False)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        
        for detection in detections:
            if detection.tag_id == self.target_id:
                corners = detection.corners
                tag_area = self._calculate_polygon_area(corners)
                frame_area = frame.shape[0] * frame.shape[1]
                
                return TagDetection(
                    found=True,
                    center_x=detection.center[0],
                    center_y=detection.center[1],
                    corners=corners,
                    tag_area=tag_area,
                    area_ratio=tag_area / frame_area,
                    tag_id=detection.tag_id
                )
        
        return TagDetection(found=False)
    
    def detect_all(self, frame: np.ndarray) -> list:
        """Detect all AprilTags in frame."""
        if self.detector is None:
            return []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        
        results = []
        frame_area = frame.shape[0] * frame.shape[1]
        
        for detection in detections:
            corners = detection.corners
            tag_area = self._calculate_polygon_area(corners)
            area_ratio = tag_area / frame_area
            
            results.append(TagDetection(
                found=True,
                center_x=detection.center[0],
                center_y=detection.center[1],
                corners=corners,
                tag_area=tag_area,
                area_ratio=area_ratio,
                tag_id=detection.tag_id
            ))
        
        return results
    
    @staticmethod
    def _calculate_polygon_area(corners: np.ndarray) -> float:
        """Calculate polygon area using shoelace formula."""
        n = len(corners)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += corners[i][0] * corners[j][1]
            area -= corners[j][0] * corners[i][1]
        return abs(area) / 2.0
