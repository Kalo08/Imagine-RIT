"""USB link to the ESP32-P4: byte transport + CRC-verified framing + latest-state cache.

Link URLs (``--link``):
    tcp://127.0.0.1:5555     tools/sim_bridge.py (no hardware)
    serial:/dev/ttyACM0      real ESP32-P4 USB CDC-ACM (also ``serial:COM7``)
    /dev/ttyACM0 | COM7      shorthand for serial:
"""
from __future__ import annotations

import socket
import threading
import time
from collections import deque
from typing import Callable

from generated import robot_protocol as rp


class _Transport:
    def read(self, n: int) -> bytes: ...
    def write(self, data: bytes) -> None: ...
    def close(self) -> None: ...


class _TcpTransport(_Transport):
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.1)

    def read(self, n: int) -> bytes:
        try:
            d = self.sock.recv(n)
        except socket.timeout:
            return b""
        if not d:
            raise ConnectionError("link closed by peer")
        return d

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class _SerialTransport(_Transport):
    def __init__(self, path: str):
        import serial  # pyserial; imported lazily so tcp-only use needs nothing

        # Baud is irrelevant for USB CDC but pyserial wants one.
        self.ser = serial.Serial(path, baudrate=115200, timeout=0.05, write_timeout=0.5)

    def read(self, n: int) -> bytes:
        waiting = self.ser.in_waiting
        return self.ser.read(max(1, min(waiting, n)))

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


def open_transport(url: str) -> _Transport:
    if url.startswith("tcp://"):
        host, _, port = url[6:].partition(":")
        return _TcpTransport(host or "127.0.0.1", int(port))
    if url.startswith("serial:"):
        url = url[7:]
    return _SerialTransport(url)


class UsbLink:
    """Owns the transport and a reader thread. Thread-safe for one writer + many readers."""

    def __init__(self, url: str):
        self.url = url
        self._t = open_transport(url)
        self._parser = rp.FrameParser()
        self._lock = threading.Lock()
        self._state: rp.StatePacket | None = None
        self._state_rx_time = 0.0
        self._state_times: deque[float] = deque(maxlen=400)
        self._state_listeners: list[Callable[[rp.StatePacket], None]] = []
        self._roi_listeners: list[Callable[[rp.RoiFrame], None]] = []
        self.rx_states = 0
        self.rx_rois = 0
        self.tx_actions = 0
        self._stop = threading.Event()
        self._err: Exception | None = None
        self._thread = threading.Thread(target=self._reader, name="usb-rx", daemon=True)
        self._thread.start()

    # ---- callbacks (invoked on the reader thread; keep them short) ----
    def add_state_listener(self, cb: Callable[[rp.StatePacket], None]) -> None:
        self._state_listeners.append(cb)

    def add_roi_listener(self, cb: Callable[[rp.RoiFrame], None]) -> None:
        self._roi_listeners.append(cb)

    # ---- reader ----
    def _reader(self) -> None:
        try:
            while not self._stop.is_set():
                data = self._t.read(4096)
                if not data:
                    continue
                for pkt in self._parser.feed(data):
                    if isinstance(pkt, rp.StatePacket):
                        now = time.monotonic()
                        with self._lock:
                            self._state, self._state_rx_time = pkt, now
                            self._state_times.append(now)
                            self.rx_states += 1
                        for cb in self._state_listeners:
                            cb(pkt)
                    elif isinstance(pkt, rp.RoiFrame):
                        self.rx_rois += 1
                        for cb in self._roi_listeners:
                            cb(pkt)
        except Exception as e:  # connection loss etc.; surfaced via .error
            self._err = e

    # ---- public API ----
    @property
    def error(self) -> Exception | None:
        return self._err

    def send_action(self, pkt: rp.ActionPacket) -> None:
        self._t.write(rp.encode_packet(pkt))
        self.tx_actions += 1

    def latest_state(self) -> tuple[rp.StatePacket | None, float]:
        """Return (state, age_seconds). age is inf if nothing was received yet."""
        with self._lock:
            if self._state is None:
                return None, float("inf")
            return self._state, time.monotonic() - self._state_rx_time

    def state_rate_hz(self) -> float:
        with self._lock:
            ts = list(self._state_times)
        if len(ts) < 2 or ts[-1] == ts[0]:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def wait_for_state(self, timeout: float = 3.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.latest_state()[0] is not None:
                return True
            if self._err:
                raise ConnectionError(str(self._err))
            time.sleep(0.01)
        return False

    @property
    def crc_errors(self) -> int:
        return self._parser.crc_errors

    @property
    def len_errors(self) -> int:
        return self._parser.len_errors

    def close(self) -> None:
        self._stop.set()
        self._t.close()
        self._thread.join(timeout=1.0)
