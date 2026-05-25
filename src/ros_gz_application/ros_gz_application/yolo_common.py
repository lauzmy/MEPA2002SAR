"""Shared helpers for YOLO-based detection nodes (NCNN export, FPS tracking, overlay)."""

import os

import cv2
from ultralytics import YOLO

FPS_OVERLAY_POS = (10, 30)
FPS_OVERLAY_COLOR = (0, 255, 0)
FPS_OVERLAY_FONT_SCALE = 1
FPS_OVERLAY_THICKNESS = 2


def ensure_ncnn(model_path, logger):
    """If `model_path` is a .pt file, export to NCNN once and return the NCNN directory."""
    if not model_path.endswith('.pt'):
        return model_path
    ncnn_dir = model_path.replace('.pt', '_ncnn_model')
    if not os.path.exists(ncnn_dir):
        logger.info(f'Exporting {model_path} to NCNN (one-time, may take a while)...')
        YOLO(model_path, task='detect').export(format='ncnn')
        logger.info(f'NCNN export saved to {ncnn_dir}')
    return ncnn_dir


class FpsTracker:
    """One-second rolling window FPS estimator. Caller drives `update(now_s)` per frame."""

    def __init__(self, now_s):
        self._frames_in_window = 0
        self._window_start_s = now_s
        self.current_fps = 0.0

    def update(self, now_s):
        self._frames_in_window += 1
        window_s = now_s - self._window_start_s
        if window_s >= 1.0:
            self.current_fps = self._frames_in_window / window_s
            self._frames_in_window = 0
            self._window_start_s = now_s


class FpsLimiter:
    """Frame throttle. `should_skip(now_s)` is True when the caller should drop the frame."""

    def __init__(self, fps_limit):
        self._fps_limit = fps_limit
        # 0.0 so the first frame always passes — initializing to monotonic() would drop frame 1.
        self._last_frame_time_s = 0.0

    def should_skip(self, now_s):
        if self._fps_limit <= 0.0:
            return False
        if now_s - self._last_frame_time_s < 1.0 / self._fps_limit:
            return True
        self._last_frame_time_s = now_s
        return False


def draw_fps_overlay(frame, fps):
    cv2.putText(frame, f'FPS: {fps:.1f}', FPS_OVERLAY_POS,
                cv2.FONT_HERSHEY_SIMPLEX, FPS_OVERLAY_FONT_SCALE,
                FPS_OVERLAY_COLOR, FPS_OVERLAY_THICKNESS)
