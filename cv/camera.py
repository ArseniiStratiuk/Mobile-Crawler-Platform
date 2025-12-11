#!/usr/bin/env python3
"""
Camera module for video capture and streaming.
"""

import cv2
import subprocess
import threading

from config import (
    CAMERA_ID, FRAME_WIDTH, FRAME_HEIGHT,
    CAMERA_FPS, CAMERA_BUFFER_SIZE,
    STREAM_HOST, STREAM_PORT
)


class Camera:
    """OpenCV camera handler."""
    
    def __init__(self, camera_id: int = CAMERA_ID,
                 width: int = FRAME_WIDTH, height: int = FRAME_HEIGHT):
        self.camera_id = camera_id
        self.desired_width = width
        self.desired_height = height
        self.cap = None
        self.actual_width = width
        self.actual_height = height
    
    def init(self) -> bool:
        print(f"Initializing camera {self.camera_id}...")
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            print(f"Cannot open camera {self.camera_id}")
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.desired_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desired_height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        
        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Resolution: {self.actual_width}x{self.actual_height}")
        return True
    
    def grab_frame(self):
        """Grab latest frame (discards buffered frames)."""
        self.cap.grab()
        return self.cap.read()
    
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    @property
    def frame_center_x(self) -> int:
        return self.actual_width // 2
    
    @property
    def frame_center_y(self) -> int:
        return self.actual_height // 2


class VideoStreamer:
    """FFmpeg TCP video streamer."""
    
    def __init__(self, width: int = FRAME_WIDTH, height: int = FRAME_HEIGHT,
                 port: int = STREAM_PORT):
        self.width = width
        self.height = height
        self.port = port
        self.ffmpeg_process = None
        self.stream_ready = threading.Event()
        self.stream_thread = None
    
    def start(self) -> bool:
        print(f"Starting stream on port {self.port}")
        self.stream_thread = threading.Thread(target=self._start_ffmpeg, daemon=True)
        self.stream_thread.start()
        print(f"Connect: ffplay -fflags nobuffer tcp://<IP>:{self.port}")
        return True
    
    def _start_ffmpeg(self):
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', '30',
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-f', 'mpegts',
            f'tcp://{STREAM_HOST}:{self.port}?listen=1'
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
            print(f"FFmpeg error: {e}")
    
    def send_frame(self, frame):
        """Send frame to stream."""
        if self.ffmpeg_process is not None and self.ffmpeg_process.poll() is None:
            try:
                self.ffmpeg_process.stdin.write(frame.tobytes())
            except:
                pass
    
    def stop(self):
        """Stop stream."""
        if self.ffmpeg_process is not None:
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
