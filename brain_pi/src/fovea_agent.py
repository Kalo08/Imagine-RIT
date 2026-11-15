"""Gaze-locked event dispatcher.

The ESP32-P4 does the fast foveated tracking; whenever its StatePacket reports
``gaze_locked`` the Pi gets a (debounced) event and can run slower, heavier
perception on the foveal crop (RoiFrame) and/or trigger audio behaviours.

Register your own hooks:

    agent = FoveaAgent(link)
    agent.on_gaze_locked(lambda ev: my_face_recognizer(ev.roi))
    agent.on_gaze_lost(lambda ev: ...)

Hooks run on a worker thread, so they may take tens of milliseconds without
disturbing the 50 Hz control loop or the USB reader.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable

from generated import robot_protocol as rp
from usb_client import UsbLink

log = logging.getLogger("fovea")


@dataclass
class GazeEvent:
    kind: str  # "locked" | "lost"
    t_monotonic: float
    esp_timestamp_us: int
    x: float
    y: float
    confidence: float
    roi: rp.RoiFrame | None  # most recent foveal crop, if any arrived


class FoveaAgent:
    def __init__(self, link: UsbLink, min_confidence: float | None = None, debounce_s: float = 0.15):
        self.min_conf = rp.GAZE["min_confidence"] if min_confidence is None else min_confidence
        self.debounce_s = debounce_s
        self._locked = False
        self._candidate_since: float | None = None
        self._last_roi: rp.RoiFrame | None = None
        self._cbs: dict[str, list[Callable[[GazeEvent], None]]] = {"locked": [], "lost": []}
        self._q: queue.Queue[GazeEvent] = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="fovea-hooks", daemon=True)
        self._worker.start()
        self.events_dispatched = 0
        link.add_state_listener(self._on_state)
        link.add_roi_listener(self._on_roi)
        # Default hooks only log; replace/extend with real vision/audio pipelines.
        self.on_gaze_locked(self._default_vision_hook)
        self.on_gaze_locked(self._default_audio_hook)

    def on_gaze_locked(self, cb: Callable[[GazeEvent], None]) -> None:
        self._cbs["locked"].append(cb)

    def on_gaze_lost(self, cb: Callable[[GazeEvent], None]) -> None:
        self._cbs["lost"].append(cb)

    # ---- state machine (reader thread; must stay cheap) ----
    def _on_roi(self, roi: rp.RoiFrame) -> None:
        self._last_roi = roi

    def _on_state(self, s: rp.StatePacket) -> None:
        now = time.monotonic()
        want = bool(s.gaze_locked) and s.gaze_confidence >= self.min_conf
        if want and not self._locked:
            if self._candidate_since is None:
                self._candidate_since = now
            elif now - self._candidate_since >= self.debounce_s:
                self._locked, self._candidate_since = True, None
                self._emit("locked", s, now)
        elif not want and self._locked:
            self._locked = False
            self._emit("lost", s, now)
        elif not want:
            self._candidate_since = None

    def _emit(self, kind: str, s: rp.StatePacket, now: float) -> None:
        ev = GazeEvent(kind, now, s.timestamp_us, s.gaze_x, s.gaze_y, s.gaze_confidence, self._last_roi)
        try:
            self._q.put_nowait(ev)
        except queue.Full:
            log.warning("fovea event queue full; dropping %s event", kind)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                ev = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            for cb in self._cbs[ev.kind]:
                try:
                    cb(ev)
                except Exception:
                    log.exception("fovea hook failed")
            self.events_dispatched += 1

    def close(self) -> None:
        self._stop.set()
        self._worker.join(timeout=1.0)

    # ---- default hooks ----
    @staticmethod
    def _default_vision_hook(ev: GazeEvent) -> None:
        roi = "no ROI yet" if ev.roi is None else f"ROI {rp.ROI_W}x{rp.ROI_H} @ t={ev.roi.timestamp_us}us"
        log.info("[vision hook] gaze locked at (%.2f, %.2f) conf=%.2f; %s", ev.x, ev.y, ev.confidence, roi)

    @staticmethod
    def _default_audio_hook(ev: GazeEvent) -> None:
        log.info("[audio hook] orienting microphones toward gaze x=%.2f", ev.x)
