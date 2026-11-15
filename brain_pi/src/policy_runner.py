"""50 Hz control loop + policy bundle loader.

Modes
-----
neutral_hold   Send NEUTRAL_HOLD every tick (ESP holds the neutral pose; its
               reflexes, gaze tracking and face UI stay live). No policy needed.
manual_jog     Hold neutral plus operator-set per-joint offsets (see cli.py `jog`).
onnx           Run a validated policy.onnx + manifest.json bundle. Motor output
               is only enabled if the manifest matches robot_spec.yaml exactly.
auto           onnx if a valid bundle exists in the policies directory, else neutral_hold.

Safety on the Pi (defence in depth; the ESP32-P4 enforces its own hard limits):
  * targets are clamped to joint limits and slew-limited to max_vel_rad_s,
  * stale telemetry (> 100 ms) or any policy error/NaN falls back to NEUTRAL_HOLD,
  * on exit a NEUTRAL_HOLD frame is sent; if the Pi dies, the ESP watchdog holds neutral.
"""
from __future__ import annotations

import enum
import hashlib
import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from generated import robot_protocol as rp
from usb_client import UsbLink

log = logging.getLogger("runner")

N = rp.NUM_JOINTS
STALE_STATE_S = 0.100
MANIFEST_SCHEMA_VERSION = 1


class Mode(str, enum.Enum):
    NEUTRAL_HOLD = "neutral_hold"
    MANUAL_JOG = "manual_jog"
    ONNX = "onnx"
    AUTO = "auto"


# =============================================================================
# Policy bundle (policy.onnx + manifest.json)
# =============================================================================
class PolicyError(Exception):
    """Raised with a multi-line message listing every contract violation found."""


# observation term name -> size (None = size must be given in the manifest)
OBS_TERM_SIZES: dict[str, int | None] = {
    "joint_pos_rel": N,        # pos - neutral_rad (rad)
    "joint_vel": N,            # rad/s
    "joint_current_norm": N,   # current_ma / max_current_ma
    "last_action": N,          # previous raw network output
    "imu_quat_wxyz": 4,
    "imu_gyro": 3,             # rad/s, body frame
    "projected_gravity": 3,    # unit gravity direction in body frame
    "gaze": 3,                 # x, y, locked(0/1)
    "command": None,           # operator/high-level command vector; manifest gives `size`
}
ACTION_MODES = ("delta_from_neutral", "absolute_rad")


@dataclass
class PolicyBundle:
    directory: Path
    manifest: dict
    terms: list[tuple[str, int, float]]  # (name, size, scale)
    obs_dim: int
    action_mode: str
    action_scale: float
    action_clip: float | None
    session: object = field(repr=False, default=None)
    input_name: str = "obs"
    output_name: str = "action"

    @property
    def name(self) -> str:
        return self.manifest.get("policy_name", self.directory.name)

    # -- discovery --
    @staticmethod
    def find(path: Path) -> Path | None:
        """`path` itself, or its newest sub-directory, that contains a manifest.json."""
        if (path / "manifest.json").is_file():
            return path
        subs = [d for d in path.glob("*/") if (d / "manifest.json").is_file()] if path.is_dir() else []
        return max(subs, key=lambda d: (d / "manifest.json").stat().st_mtime) if subs else None

    # -- validation + load --
    @classmethod
    def load(cls, directory: Path, create_session: bool = True) -> "PolicyBundle":
        errs: list[str] = []
        mpath = directory / "manifest.json"
        try:
            m = json.loads(mpath.read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            raise PolicyError(f"cannot read {mpath}: {e}") from e

        if m.get("schema_version") != MANIFEST_SCHEMA_VERSION:
            errs.append(f"schema_version must be {MANIFEST_SCHEMA_VERSION}, got {m.get('schema_version')!r}")
        if m.get("num_joints") != N:
            errs.append(f"num_joints={m.get('num_joints')!r} but robot_spec.yaml has {N}")
        names = m.get("joint_names")
        if names != list(rp.JOINT_NAMES):
            errs.append(f"joint_names mismatch.\n    manifest: {names}\n    robot   : {list(rp.JOINT_NAMES)}")
        if m.get("control_hz") != rp.ACTION_HZ:
            errs.append(f"control_hz={m.get('control_hz')!r} but robot action_hz is {rp.ACTION_HZ}")
        if "spec_hash" in m and m["spec_hash"] != rp.SPEC_HASH:
            errs.append(f"spec_hash {m['spec_hash']} != current {rp.SPEC_HASH} (robot_spec changed since training)")

        onnx_path = directory / m.get("onnx_file", "policy.onnx")
        if not onnx_path.is_file():
            errs.append(f"missing ONNX file {onnx_path.name}")
        elif "onnx_sha256" in m:
            digest = hashlib.sha256(onnx_path.read_bytes()).hexdigest()
            if digest != m["onnx_sha256"]:
                errs.append("onnx_sha256 does not match policy.onnx (wrong/corrupted file)")

        obs = m.get("observation") or {}
        terms: list[tuple[str, int, float]] = []
        for t in obs.get("terms", []):
            nm = t.get("name")
            if nm not in OBS_TERM_SIZES:
                errs.append(f"unknown observation term {nm!r}; allowed: {sorted(OBS_TERM_SIZES)}")
                continue
            size = OBS_TERM_SIZES[nm] if OBS_TERM_SIZES[nm] is not None else t.get("size")
            if not isinstance(size, int) or size < 1:
                errs.append(f"observation term {nm!r} needs integer size >= 1")
                continue
            if OBS_TERM_SIZES[nm] is not None and t.get("size", size) != size:
                errs.append(f"observation term {nm!r} has fixed size {size}, manifest says {t['size']}")
            terms.append((nm, size, float(t.get("scale", 1.0))))
        if not terms:
            errs.append("observation.terms is empty")
        obs_dim = sum(s for _, s, _ in terms)
        if obs.get("dim") != obs_dim:
            errs.append(f"observation.dim={obs.get('dim')!r} but terms sum to {obs_dim}")

        act = m.get("action") or {}
        if act.get("mode") not in ACTION_MODES:
            errs.append(f"action.mode must be one of {ACTION_MODES}")
        scale = act.get("scale", 1.0)
        clip = act.get("clip")
        if not isinstance(scale, (int, float)) or not math.isfinite(scale):
            errs.append("action.scale must be a finite number")
        if clip is not None and (not isinstance(clip, (int, float)) or clip <= 0):
            errs.append("action.clip must be a positive number or omitted")

        bundle = cls(directory, m, terms, obs_dim, act.get("mode", ""), float(scale or 1.0),
                     float(clip) if clip else None,
                     input_name=obs.get("input_name", "obs"), output_name=act.get("output_name", "action"))

        if create_session and not errs and onnx_path.is_file():
            try:
                import numpy  # noqa: F401
                import onnxruntime as ort
            except ImportError:
                errs.append("onnxruntime/numpy not installed: `uv sync --extra onnx`")
            else:
                sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
                i0, o0 = sess.get_inputs()[0], sess.get_outputs()[0]
                if i0.name != bundle.input_name:
                    errs.append(f"ONNX input is named {i0.name!r}, manifest says {bundle.input_name!r}")
                if isinstance(i0.shape[-1], int) and i0.shape[-1] != obs_dim:
                    errs.append(f"ONNX input width {i0.shape[-1]} != observation.dim {obs_dim}")
                if o0.name != bundle.output_name:
                    errs.append(f"ONNX output is named {o0.name!r}, manifest says {bundle.output_name!r}")
                if isinstance(o0.shape[-1], int) and o0.shape[-1] != N:
                    errs.append(f"ONNX output width {o0.shape[-1]} != NUM_JOINTS {N}")
                bundle.session = sess
        if errs:
            raise PolicyError(f"policy bundle {directory} REJECTED:\n  - " + "\n  - ".join(errs))
        return bundle

    # -- inference --
    def observe(self, s: rp.StatePacket, last_action: list[float], command: list[float]):
        import numpy as np

        w, x, y, z = s.quat_wxyz
        grav = (-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y)))
        src = {
            "joint_pos_rel": [j.pos_rad - rp.JOINT_NEUTRAL_RAD[i] for i, j in enumerate(s.joints)],
            "joint_vel": [j.vel_rad_s for j in s.joints],
            "joint_current_norm": [j.current_ma / rp.JOINT_MAX_CURRENT_MA[i] for i, j in enumerate(s.joints)],
            "last_action": last_action,
            "imu_quat_wxyz": list(s.quat_wxyz),
            "imu_gyro": list(s.gyro_rad_s),
            "projected_gravity": list(grav),
            "gaze": [s.gaze_x, s.gaze_y, float(s.gaze_locked)],
            "command": command,
        }
        vec: list[float] = []
        for name, size, scale in self.terms:
            v = src[name]
            vec.extend(float(a) * scale for a in (v + [0.0] * size)[:size])
        return np.asarray(vec, dtype=np.float32)[None, :]

    def infer(self, obs) -> tuple[list[float], list[float]]:
        """Returns (raw network output, joint position targets in rad)."""
        out = self.session.run([self.output_name], {self.input_name: obs})[0].reshape(-1)
        if out.shape[0] != N or not all(math.isfinite(float(a)) for a in out):
            raise PolicyError(f"policy produced invalid output (shape {out.shape}, finite={bool(out.size)})")
        raw = [float(a) for a in out]
        used = [max(-self.action_clip, min(self.action_clip, a)) if self.action_clip else a for a in raw]
        if self.action_mode == "delta_from_neutral":
            tgt = [rp.JOINT_NEUTRAL_RAD[i] + a * self.action_scale for i, a in enumerate(used)]
        else:
            tgt = [a * self.action_scale for a in used]
        return raw, tgt


# =============================================================================
# 50 Hz runner
# =============================================================================
@dataclass
class RunStats:
    ticks: int = 0
    overruns: int = 0
    fallbacks: int = 0
    max_tick_ms: float = 0.0
    last_log: float = 0.0


class PolicyRunner:
    def __init__(self, link: UsbLink, mode: Mode, bundle: PolicyBundle | None = None,
                 hz: int = rp.ACTION_HZ, gaze_override: bool = False):
        if mode is Mode.ONNX and bundle is None:
            raise PolicyError("onnx mode requires a validated policy bundle")
        self.link, self.mode, self.bundle, self.hz = link, mode, bundle, hz
        self.gaze_override = gaze_override
        self.stats = RunStats()
        self.command = [0.0] * 8
        self._seq = 0
        self._estop = False
        self._jog = [0.0] * N  # offsets from neutral (rad)
        self._jog_lock = threading.Lock()
        self._last_raw = [0.0] * N
        self._last_sent: list[float] | None = None
        self._policy_failed = False
        self.stop_event = threading.Event()

    # ---- operator controls ----
    def set_jog(self, joint: int, offset_rad: float) -> None:
        with self._jog_lock:
            self._jog[joint] = offset_rad

    def jog_offsets(self) -> list[float]:
        with self._jog_lock:
            return list(self._jog)

    def estop(self) -> None:
        self._estop = True

    def rearm(self) -> None:
        self._estop = False

    # ---- target computation ----
    def _targets(self, s: rp.StatePacket | None, stale: bool) -> tuple[rp.ControlMode, list[float]]:
        neutral = list(rp.JOINT_NEUTRAL_RAD)
        if self._estop:
            return rp.ControlMode.ESTOP, neutral
        if self.mode is Mode.NEUTRAL_HOLD or s is None or stale:
            if stale and self.mode is not Mode.NEUTRAL_HOLD:
                self.stats.fallbacks += 1
            return rp.ControlMode.NEUTRAL_HOLD, neutral
        if self.mode is Mode.MANUAL_JOG:
            off = self.jog_offsets()
            return rp.ControlMode.ONNX_POLICY, [neutral[i] + off[i] for i in range(N)]
        # ONNX
        if self._policy_failed:
            return rp.ControlMode.NEUTRAL_HOLD, neutral
        try:
            obs = self.bundle.observe(s, self._last_raw, self.command)
            self._last_raw, tgt = self.bundle.infer(obs)
            return rp.ControlMode.ONNX_POLICY, tgt
        except Exception:
            log.exception("policy failure -> latching NEUTRAL_HOLD (restart runner to retry)")
            self._policy_failed = True
            self.stats.fallbacks += 1
            return rp.ControlMode.NEUTRAL_HOLD, neutral

    def _limit(self, tgt: list[float], s: rp.StatePacket | None, dt: float) -> list[float]:
        """Clamp to joint range and slew-limit relative to the previous command."""
        if self._last_sent is None:
            self._last_sent = [j.pos_rad for j in s.joints] if s else list(rp.JOINT_NEUTRAL_RAD)
        out = []
        for i, t in enumerate(tgt):
            if not math.isfinite(t):
                t = rp.JOINT_NEUTRAL_RAD[i]
            t = max(rp.JOINT_MIN_RAD[i], min(rp.JOINT_MAX_RAD[i], t))
            step = rp.JOINT_MAX_VEL_RAD_S[i] * dt
            prev = self._last_sent[i]
            out.append(prev + max(-step, min(step, t - prev)))
        self._last_sent = out
        return out

    # ---- main loop ----
    def run(self, duration: float | None = None, log_every: float = 1.0,
            on_stats: Callable[[dict], None] | None = None) -> dict:
        period = 1.0 / self.hz
        t0 = time.perf_counter()
        deadline = t0
        rx0 = self.link.rx_states
        tx0 = self.link.tx_actions
        win_t, win_rx, win_tx = t0, rx0, tx0
        try:
            while not self.stop_event.is_set():
                tick_start = time.perf_counter()
                if duration is not None and tick_start - t0 >= duration:
                    break
                if self.link.error:
                    raise ConnectionError(f"USB link lost: {self.link.error}")
                s, age = self.link.latest_state()
                stale = s is None or age > STALE_STATE_S
                mode, tgt = self._targets(s, stale)
                tgt = self._limit(tgt, s, period)
                self._seq = (self._seq + 1) & 0xFFFFFFFF
                self.link.send_action(rp.ActionPacket(
                    sequence_id=self._seq, control_mode=int(mode),
                    gaze_override=1 if self.gaze_override else 0, target_pos_rad=tgt))
                self.stats.ticks += 1

                now = time.perf_counter()
                self.stats.max_tick_ms = max(self.stats.max_tick_ms, (now - tick_start) * 1e3)
                if now - win_t >= log_every:
                    rec = self._snapshot(s, now - win_t, self.link.rx_states - win_rx, self.link.tx_actions - win_tx, mode)
                    win_t, win_rx, win_tx = now, self.link.rx_states, self.link.tx_actions
                    (on_stats or self._print_stats)(rec)

                deadline += period
                sleep = deadline - time.perf_counter()
                if sleep < -period:  # fell more than a whole period behind: resync, count it
                    self.stats.overruns += 1
                    deadline = time.perf_counter()
                elif sleep > 0.0008:
                    time.sleep(sleep - 0.0006)
                while time.perf_counter() < deadline:  # short spin for jitter
                    pass
        finally:
            try:  # leave the robot in a safe, torque-on neutral hold
                self._seq = (self._seq + 1) & 0xFFFFFFFF
                self.link.send_action(rp.ActionPacket(sequence_id=self._seq, control_mode=int(rp.ControlMode.NEUTRAL_HOLD),
                                                      target_pos_rad=list(rp.JOINT_NEUTRAL_RAD)))
            except Exception:
                pass
        elapsed = time.perf_counter() - t0
        s, _ = self.link.latest_state()
        return {
            "elapsed_s": round(elapsed, 3),
            "tx_actions": self.link.tx_actions - tx0,
            "tx_hz": round((self.link.tx_actions - tx0) / elapsed, 1) if elapsed else 0.0,
            "rx_states": self.link.rx_states - rx0,
            "rx_hz": round((self.link.rx_states - rx0) / elapsed, 1) if elapsed else 0.0,
            "crc_errors": self.link.crc_errors,
            "len_errors": self.link.len_errors,
            "overruns": self.stats.overruns,
            "fallbacks": self.stats.fallbacks,
            "max_tick_ms": round(self.stats.max_tick_ms, 3),
            "last_ack_seq": s.ack_sequence_id if s else None,
            "last_sent_seq": self._seq,
            "mode": self.mode.value,
        }

    def _snapshot(self, s, dt, drx, dtx, mode) -> dict:
        return {
            "mode_sent": mode.name, "state_hz": round(drx / dt, 1), "action_hz": round(dtx / dt, 1),
            "crc_err": self.link.crc_errors, "ack_lag": ((self._seq - s.ack_sequence_id) & 0xFFFFFFFF) if s else None,
            "reflex": f"{s.reflex_mask:#06x}" if s else None,
            "gaze_locked": bool(s.gaze_locked) if s else None,
            "quat": [round(q, 3) for q in s.quat_wxyz] if s else None,
        }

    @staticmethod
    def _print_stats(rec: dict) -> None:
        print("  ".join(f"{k}={v}" for k, v in rec.items()), flush=True)


def resolve_mode(requested: Mode, policies_dir: Path) -> tuple[Mode, PolicyBundle | None]:
    """Turn --mode into (effective mode, validated bundle or None). Never enables ONNX unvalidated."""
    if requested in (Mode.NEUTRAL_HOLD, Mode.MANUAL_JOG):
        return requested, None
    found = PolicyBundle.find(policies_dir)
    if found is None:
        if requested is Mode.ONNX:
            raise PolicyError(f"no manifest.json found under {policies_dir}")
        log.warning("no policy bundle in %s -> neutral_hold fallback", policies_dir)
        return Mode.NEUTRAL_HOLD, None
    try:
        return Mode.ONNX, PolicyBundle.load(found)
    except PolicyError:
        if requested is Mode.ONNX:
            raise
        log.exception("policy bundle rejected -> neutral_hold fallback")
        return Mode.NEUTRAL_HOLD, None
