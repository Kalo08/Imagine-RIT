#!/usr/bin/env python3
"""Software stand-in for the ESP32-P4: speaks the real USB binary protocol.

    python tools/sim_bridge.py                    # TCP server on 127.0.0.1:5555
    python tools/sim_bridge.py --pty               # POSIX only: prints /dev/pts/N to open with `serial:`

Emulates (at 200 Hz): synthetic IMU, first-order servo echo of Pi targets with hard-limit
clamping, current estimate, action-timeout watchdog, latched ESTOP, a sweeping gaze target
that locks periodically, and 10 Hz foveal ROI frames. It exists to exercise brain_pi's
protocol, timing and mode logic; it is NOT a physics simulation.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import select
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "brain_pi" / "src"))
from generated import robot_protocol as rp  # noqa: E402

N = rp.NUM_JOINTS
DT = 1.0 / rp.STATE_HZ
TAU = 0.04  # servo first-order lag (s)


class SocketEndpoint:
    def __init__(self, port: int, host: str):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn: socket.socket | None = None
        print(f"[sim_bridge] listening on tcp://{host}:{port}  (NUM_JOINTS={N})", flush=True)

    def poll(self) -> bytes:
        if self.conn is None:
            try:
                self.conn, addr = self.srv.accept()
                self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.conn.setblocking(False)
                print(f"[sim_bridge] client connected {addr}", flush=True)
            except (BlockingIOError, OSError):
                return b""
        try:
            d = self.conn.recv(4096)
        except (BlockingIOError, InterruptedError):
            return b""
        except OSError:
            d = b""
        if not d:
            print("[sim_bridge] client disconnected", flush=True)
            self.conn.close()
            self.conn = None
        return d

    def write(self, data: bytes) -> None:
        if self.conn is None:
            return
        try:
            self.conn.sendall(data)
        except (BlockingIOError, OSError):
            pass  # a slow/gone client just misses frames, like a real lossy link


class PtyEndpoint:
    def __init__(self):
        if not hasattr(os, "openpty"):
            sys.exit("--pty needs a POSIX OS; use the default TCP endpoint on Windows")
        import tty

        self.master, slave = os.openpty()
        tty.setraw(self.master)
        os.set_blocking(self.master, False)
        print(f"[sim_bridge] PTY ready: use --link serial:{os.ttyname(slave)}", flush=True)
        self._slave = slave  # keep open

    def poll(self) -> bytes:
        try:
            return os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return b""

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except (BlockingIOError, OSError):
            pass


class SimRobot:
    def __init__(self, drop: float = 0.0):
        self.t = 0.0
        self.pos = list(rp.JOINT_NEUTRAL_RAD)
        self.vel = [0.0] * N
        self.cur = [0.0] * N
        self.target = list(rp.JOINT_NEUTRAL_RAD)
        self.mode = rp.ControlMode.NEUTRAL_HOLD
        self.estop_latched = False
        self.gaze_override = 0
        self.ack = 0
        self.last_action_t = None  # monotonic time of last good action
        self.reflex = 0
        self.lock_frames = 0
        self.drop = drop
        self.quat_prev = (1.0, 0.0, 0.0, 0.0)
        self.parser = rp.FrameParser()
        self.rng = random.Random(1)

    # --- inbound ---
    def on_bytes(self, data: bytes, now: float) -> None:
        for pkt in self.parser.feed(data):
            if not isinstance(pkt, rp.ActionPacket):
                continue
            self.ack = pkt.sequence_id
            self.last_action_t = now
            self.gaze_override = pkt.gaze_override
            try:
                mode = rp.ControlMode(pkt.control_mode)
            except ValueError:
                continue  # unknown mode: ignore the whole packet
            if mode == rp.ControlMode.ESTOP:
                self.estop_latched = True
            elif mode == rp.ControlMode.NEUTRAL_HOLD:
                self.estop_latched = False  # explicit re-arm, as the real firmware
            elif self.estop_latched:
                mode = rp.ControlMode.ESTOP
            self.mode = rp.ControlMode.ESTOP if self.estop_latched else mode
            if self.mode == rp.ControlMode.ONNX_POLICY:
                self.target = list(pkt.target_pos_rad)

    # --- physics-ish step ---
    def step(self, now: float) -> None:
        self.t += DT
        self.reflex = 0
        link_dead = self.last_action_t is None or (now - self.last_action_t) * 1e3 > rp.ACTION_TIMEOUT_MS
        mode = self.mode
        if link_dead and mode != rp.ControlMode.ESTOP:
            mode = rp.ControlMode.NEUTRAL_HOLD
            self.reflex |= rp.ReflexBit.WATCHDOG_HOLD
        want = list(rp.JOINT_NEUTRAL_RAD) if mode == rp.ControlMode.NEUTRAL_HOLD else list(self.target)
        a = 1.0 - math.exp(-DT / TAU)
        for i in range(N):
            if mode == rp.ControlMode.ESTOP:
                self.cur[i] = 0.0
                self.vel[i] = 0.0
                continue  # torque off: joint stays where it is
            lo, hi = rp.JOINT_MIN_RAD[i], rp.JOINT_MAX_RAD[i]
            tgt = want[i]
            if not math.isfinite(tgt) or tgt < lo or tgt > hi:
                self.reflex |= rp.ReflexBit.LIMIT_CLAMP
                tgt = min(hi, max(lo, tgt if math.isfinite(tgt) else rp.JOINT_NEUTRAL_RAD[i]))
            new = self.pos[i] + (tgt - self.pos[i]) * a
            v = (new - self.pos[i]) / DT
            self.cur[i] = 120.0 * abs(v) + self.rng.gauss(0, 4.0)  # mA: load proportional to speed + noise
            self.vel[i], self.pos[i] = v, new
        # neck joints follow the synthetic gaze unless the Pi overrides
        gx, gy, _, _ = self.gaze()
        if mode != rp.ControlMode.ESTOP and not self.gaze_override:
            for i in rp.NECK_JOINT_INDICES:
                tgt = gx * 0.4 if "yaw" in rp.JOINT_NAMES[i] else gy * 0.3
                self.pos[i] += (tgt - self.pos[i]) * a
        # occasional looming flinch so bench runs show reflex bits
        if self.t > 1.0 and (self.t % 7.0) < 0.1:
            self.reflex |= rp.ReflexBit.LOOMING_FLINCH

    def gaze(self):
        gx = 0.8 * math.sin(self.t * 0.7)
        gy = 0.5 * math.sin(self.t * 0.5 + 1.0)
        locked = (self.t % 6.0) > 3.0
        if locked:
            gx, gy = gx * 0.05, gy * 0.05
        return gx, gy, 0.9 if locked else 0.6, locked

    def imu(self):
        pitch = 0.05 * math.sin(self.t * 1.3)
        roll = 0.03 * math.sin(self.t * 0.9)
        cp, sp, cr, sr = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(roll / 2), math.sin(roll / 2)
        q = (cr * cp, sr * cp, cr * sp, -sr * sp)
        gyro = [0.03 * 0.9 * math.cos(self.t * 0.9), 0.05 * 1.3 * math.cos(self.t * 1.3), 0.0]
        return q, gyro

    # --- outbound ---
    def state_packet(self, now_us: int) -> rp.StatePacket:
        q, gyro = self.imu()
        gx, gy, conf, locked = self.gaze()
        s = rp.StatePacket(timestamp_us=now_us, ack_sequence_id=self.ack, active_mode=int(
            rp.ControlMode.ESTOP if self.estop_latched else self.mode),
            quat_wxyz=list(q), gyro_rad_s=gyro, reflex_mask=int(self.reflex),
            gaze_x=gx, gaze_y=gy, gaze_confidence=conf, gaze_locked=1 if locked else 0)
        for i in range(N):
            j = s.joints[i]
            j.pos_rad, j.vel_rad_s = self.pos[i], self.vel[i]
            j.current_ma = int(max(-32768, min(32767, self.cur[i])))
        return s

    def roi_packet(self, now_us: int) -> rp.RoiFrame:
        gx, gy, _, _ = self.gaze()
        cx, cy = int((gx * 0.5 + 0.5) * rp.ROI_W), int((gy * 0.5 + 0.5) * rp.ROI_H)
        px = bytearray(rp.ROI_W * rp.ROI_H)
        for y in range(rp.ROI_H):
            for x in range(rp.ROI_W):
                px[y * rp.ROI_W + x] = min(255, 40 + 215 * int((x - cx) ** 2 + (y - cy) ** 2 < 100))
        return rp.RoiFrame(timestamp_us=now_us, center_x=gx, center_y=gy, pixels=bytes(px))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--pty", action="store_true", help="expose a pseudo-terminal instead of TCP (POSIX)")
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of outbound state frames to drop (0..1)")
    ap.add_argument("--corrupt", type=float, default=0.0, help="fraction of outbound frames with a flipped bit")
    args = ap.parse_args()

    ep = PtyEndpoint() if args.pty else SocketEndpoint(args.port, args.host)
    robot = SimRobot()
    rng = random.Random(7)
    t0 = time.perf_counter()
    next_t = t0
    tick = 0
    try:
        while True:
            now = time.perf_counter()
            data = ep.poll()
            if data:
                robot.on_bytes(data, now)
            robot.step(now)
            now_us = int((now - t0) * 1e6)
            if rng.random() >= args.drop:
                frame = bytearray(rp.encode_packet(robot.state_packet(now_us)))
                if rng.random() < args.corrupt:
                    frame[rng.randrange(len(frame))] ^= 0x10
                ep.write(bytes(frame))
            if tick % (rp.STATE_HZ // 10) == 0:
                ep.write(rp.encode_packet(robot.roi_packet(now_us)))
            tick += 1
            next_t += DT
            sleep = next_t - time.perf_counter()
            if sleep < -0.1:  # host stalled; don't burst-catch-up
                next_t = time.perf_counter()
            elif sleep > 0.0008:
                time.sleep(sleep - 0.0006)
            while time.perf_counter() < next_t:
                pass
    except KeyboardInterrupt:
        print("\n[sim_bridge] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
