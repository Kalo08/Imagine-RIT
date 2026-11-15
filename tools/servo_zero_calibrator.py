#!/usr/bin/env python3
"""Measure servo zero offsets and write shared/calibration.yaml.

Procedure
  1. Tool latches ESTOP on the ESP32-P4 (torque OFF -> joints can be moved by hand).
  2. You hold the robot in its MECHANICAL ZERO pose (the pose where every joint should
     read its `neutral_rad` in robot_spec.yaml), e.g. with a jig.
  3. Tool averages ~1 s of telemetry and computes, per joint:
         new_offset = measured_pos + current_offset - neutral_rad
     (the firmware reports  pos = raw_servo_angle - offset, so this makes pos == neutral).
  4. Writes shared/calibration.yaml.  Then run `make generate`, rebuild and reflash.

The robot is intentionally left in latched ESTOP afterwards: re-arming now would drive the
joints using the OLD offsets. Reflash (or power-cycle) to continue.

    python tools/servo_zero_calibrator.py --link serial:/dev/ttyACM0
    python tools/servo_zero_calibrator.py --link tcp://127.0.0.1:5555 --yes --dry-run   # try against sim_bridge
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "brain_pi" / "src"))
from generated import robot_protocol as rp  # noqa: E402
from usb_client import UsbLink  # noqa: E402

CALIB = ROOT / "shared" / "calibration.yaml"
MOVING_STD_RAD = 0.01


def send_estop(link: UsbLink, seq: int) -> None:
    link.send_action(rp.ActionPacket(sequence_id=seq, control_mode=int(rp.ControlMode.ESTOP)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="tcp://127.0.0.1:5555")
    ap.add_argument("--seconds", type=float, default=1.0, help="averaging window")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompt (CI / sim)")
    ap.add_argument("--dry-run", action="store_true", help="print offsets, do not write calibration.yaml")
    args = ap.parse_args()

    link = UsbLink(args.link)
    if not link.wait_for_state(3.0):
        print("no telemetry from device", file=sys.stderr)
        return 1
    seq = 0xC0000000
    for _ in range(10):  # repeated: make sure the latch is taken even if a frame is lost
        seq += 1
        send_estop(link, seq)
        time.sleep(0.02)
    time.sleep(0.2)
    s, _ = link.latest_state()
    if s.active_mode != int(rp.ControlMode.ESTOP):
        print(f"device did not enter ESTOP (active_mode={s.active_mode}); refusing to continue", file=sys.stderr)
        return 1

    print("Torque is OFF. Hold every joint at its mechanical zero pose.")
    if not args.yes:
        input("Press Enter when the robot is held still in the zero pose... ")

    samples: list[list[float]] = [[] for _ in range(rp.NUM_JOINTS)]
    last_ts, end = -1, time.monotonic() + args.seconds
    while time.monotonic() < end:
        seq += 1
        send_estop(link, seq)  # keep latch fresh
        st, _ = link.latest_state()
        if st and st.timestamp_us != last_ts:
            last_ts = st.timestamp_us
            for i, j in enumerate(st.joints):
                samples[i].append(j.pos_rad)
        time.sleep(0.005)
    link.close()

    if min(len(x) for x in samples) < 20:
        print("too few samples collected", file=sys.stderr)
        return 1

    offsets, moving = {}, []
    print(f"{'joint':16s} {'measured':>9s} {'std':>7s} {'old_off':>8s} {'new_off':>8s}")
    for i, name in enumerate(rp.JOINT_NAMES):
        mean, sd = statistics.fmean(samples[i]), statistics.pstdev(samples[i])
        new = mean + rp.JOINT_ZERO_OFFSET_RAD[i] - rp.JOINT_NEUTRAL_RAD[i]
        offsets[name] = round(new, 5)
        if sd > MOVING_STD_RAD:
            moving.append(name)
        print(f"{name:16s} {mean:+9.4f} {sd:7.4f} {rp.JOINT_ZERO_OFFSET_RAD[i]:+8.4f} {new:+8.4f}")
    if moving:
        print(f"ABORT: joints still moving (std > {MOVING_STD_RAD} rad): {moving}. Hold steadier and retry.", file=sys.stderr)
        return 1
    if args.dry_run:
        print("dry run: calibration.yaml not written")
        return 0
    header = "# Written by tools/servo_zero_calibrator.py. Run `make generate` + reflash to apply.\n"
    CALIB.write_text(header + yaml.safe_dump({"zero_offset_rad": offsets}, sort_keys=False), encoding="utf-8")
    print(f"wrote {CALIB.relative_to(ROOT)}.  Next: make generate && rebuild/reflash firmware (robot stays in ESTOP until then).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
