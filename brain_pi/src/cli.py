"""robot-brain command line.

    python -m cli run --mode neutral_hold --link tcp://127.0.0.1:5555 --duration 5
    python -m cli jog --link serial:/dev/ttyACM0
    python -m cli validate-policy policies/
    python -m cli monitor | estop | info
"""
from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path

from generated import robot_protocol as rp

REPO = Path(__file__).resolve().parents[2]
DEFAULT_POLICIES = Path(__file__).resolve().parents[1] / "policies"
DEFAULT_LINK = "tcp://127.0.0.1:5555"


def check_bindings_fresh() -> None:
    """Warn if shared/*.yaml changed since `make generate` (stale NUM_JOINTS is dangerous)."""
    import hashlib

    shared = REPO / "shared"
    files = [shared / n for n in ("robot_spec.yaml", "protocol.yaml", "calibration.yaml")]
    if not (shared / "robot_spec.yaml").exists():
        return  # deployed on a Pi without the monorepo; nothing to compare
    h = hashlib.sha256()
    for p in files:
        if p.exists():
            h.update(p.read_bytes())
    if h.hexdigest()[:16] != rp.SPEC_HASH:
        sys.exit("generated bindings are STALE vs shared/*.yaml -- run `make generate` first")


def _link(args):
    from usb_client import UsbLink

    link = UsbLink(args.link)
    if not link.wait_for_state(3.0):
        link.close()
        sys.exit(f"no StatePacket from {args.link} within 3 s (is the ESP32-P4 / sim_bridge running?)")
    return link


def cmd_run(args) -> int:
    from fovea_agent import FoveaAgent
    from policy_runner import Mode, PolicyError, PolicyRunner, resolve_mode

    try:
        mode, bundle = resolve_mode(Mode(args.mode), Path(args.policies))
    except PolicyError as e:
        print(e, file=sys.stderr)
        return 3
    link = _link(args)
    agent = FoveaAgent(link)
    runner = PolicyRunner(link, mode, bundle, gaze_override=args.gaze_override)
    signal.signal(signal.SIGINT, lambda *_: runner.stop_event.set())
    print(f"[run] N={rp.NUM_JOINTS} mode={mode.value} policy={bundle.name if bundle else '-'} link={args.link}", flush=True)
    try:
        summary = runner.run(duration=args.duration)
    finally:
        agent.close()
        link.close()
    print("SUMMARY " + json.dumps(summary), flush=True)
    return 0


def cmd_jog(args) -> int:
    from policy_runner import Mode, PolicyRunner

    link = _link(args)
    runner = PolicyRunner(link, Mode.MANUAL_JOG)
    t = threading.Thread(target=lambda: runner.run(log_every=3600), daemon=True)
    t.start()
    names = list(rp.JOINT_NAMES)
    print("manual_jog: commands  `set <joint name|index> <offset_rad>`  `zero`  `show`  `estop`  `rearm`  `quit`")
    print("joints:", ", ".join(f"{i}:{n}" for i, n in enumerate(names)))
    try:
        for line in sys.stdin:
            p = line.split()
            if not p:
                continue
            if p[0] == "quit":
                break
            if p[0] == "estop":
                runner.estop()
            elif p[0] == "rearm":
                runner.rearm()
            elif p[0] == "zero":
                for i in range(rp.NUM_JOINTS):
                    runner.set_jog(i, 0.0)
            elif p[0] == "show":
                s, _ = link.latest_state()
                for i, n in enumerate(names):
                    pos = f"{s.joints[i].pos_rad:+.3f}" if s else "?"
                    print(f"  {i} {n:16s} jog={runner.jog_offsets()[i]:+.3f} pos={pos}")
            elif p[0] == "set" and len(p) == 3:
                try:
                    idx = int(p[1]) if p[1].isdigit() else names.index(p[1])
                    runner.set_jog(idx, float(p[2]))
                except (ValueError, IndexError):
                    print("  bad joint or value")
            else:
                print("  ?")
    finally:
        runner.stop_event.set()
        t.join(timeout=1)
        link.close()
    return 0


def cmd_validate(args) -> int:
    from policy_runner import PolicyBundle, PolicyError

    d = PolicyBundle.find(Path(args.path))
    if d is None:
        print(f"no manifest.json under {args.path}", file=sys.stderr)
        return 3
    try:
        b = PolicyBundle.load(d, create_session=not args.no_session)
    except PolicyError as e:
        print(e, file=sys.stderr)
        return 3
    print(f"OK  {b.name}: obs_dim={b.obs_dim} terms={[t[0] for t in b.terms]} action={b.action_mode}")
    return 0


def cmd_monitor(args) -> int:
    link = _link(args)
    try:
        while True:
            s, age = link.latest_state()
            print(f"rx={link.state_rate_hz():6.1f}Hz age={age*1e3:5.1f}ms mode={s.active_mode} reflex={s.reflex_mask:#06x} "
                  f"gaze=({s.gaze_x:+.2f},{s.gaze_y:+.2f},c={s.gaze_confidence:.2f},L={s.gaze_locked}) "
                  f"pos={[round(j.pos_rad, 2) for j in s.joints]}", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        link.close()


def cmd_estop(args) -> int:
    link = _link(args)
    for i in range(10):  # repeat: the link is lossy by design (stale frames are dropped)
        link.send_action(rp.ActionPacket(sequence_id=0xFFFF0000 + i, control_mode=int(rp.ControlMode.ESTOP)))
        time.sleep(0.02)
    s, _ = link.latest_state()
    print(f"ESTOP sent; ESP reports active_mode={s.active_mode if s else '?'} (2 == ESTOP)")
    link.close()
    return 0


def cmd_info(_args) -> int:
    print(f"robot={rp.ROBOT_NAME} NUM_JOINTS={rp.NUM_JOINTS} spec_hash={rp.SPEC_HASH}")
    for i, n in enumerate(rp.JOINT_NAMES):
        print(f"  {i}: {n:16s} id={rp.JOINT_IDS[i]:3d} {rp.JOINT_ROLES[i]:6s} "
              f"[{rp.JOINT_MIN_RAD[i]:+.2f},{rp.JOINT_MAX_RAD[i]:+.2f}] neutral={rp.JOINT_NEUTRAL_RAD[i]:+.2f}")
    print(f"packets: StatePacket={rp.StatePacket.SIZE}B ActionPacket={rp.ActionPacket.SIZE}B RoiFrame={rp.RoiFrame.SIZE}B")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="robot-brain")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def link_arg(p):
        p.add_argument("--link", default=DEFAULT_LINK, help="tcp://host:port | serial:/dev/ttyACM0 | COM7")

    p = sub.add_parser("run", help="50 Hz control loop")
    link_arg(p)
    p.add_argument("--mode", choices=["neutral_hold", "manual_jog", "onnx", "auto"], default="auto")
    p.add_argument("--policies", default=str(DEFAULT_POLICIES))
    p.add_argument("--duration", type=float, default=None, help="seconds (default: until Ctrl-C)")
    p.add_argument("--gaze-override", action="store_true", help="Pi drives neck joints; disable ESP gaze tracking")
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("jog", help="interactive manual joint jogging")
    link_arg(p)
    p.set_defaults(fn=cmd_jog)

    p = sub.add_parser("validate-policy", help="check a policy bundle against robot_spec.yaml")
    p.add_argument("path", nargs="?", default=str(DEFAULT_POLICIES))
    p.add_argument("--no-session", action="store_true", help="skip loading onnxruntime (manifest checks only)")
    p.set_defaults(fn=cmd_validate)

    for name, fn, h in (("monitor", cmd_monitor, "print live telemetry"), ("estop", cmd_estop, "send ESTOP")):
        p = sub.add_parser(name, help=h)
        link_arg(p)
        p.set_defaults(fn=fn)

    p = sub.add_parser("info", help="print joint table / packet sizes")
    p.set_defaults(fn=cmd_info)

    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    check_bindings_fresh()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
