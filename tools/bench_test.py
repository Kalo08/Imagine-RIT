#!/usr/bin/env python3
"""`make bench-test`: run sim_bridge + brain_pi (neutral_hold) and check the link end to end.

Works on Windows/macOS/Linux without make. Exit code 0 only if:
  * action packets flowed at ~50 Hz and state packets at >= 150 Hz,
  * zero CRC / length errors,
  * the ESP side acknowledged our latest sequence id (round trip works).
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "brain_pi" / "src"


def wait_port(port: int, timeout: float = 5.0) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--min-state-hz", type=float, default=150.0)
    args = ap.parse_args()

    env = dict(os.environ, PYTHONPATH=str(SRC), PYTHONUNBUFFERED="1")
    bridge = subprocess.Popen([sys.executable, str(ROOT / "tools" / "sim_bridge.py"), "--port", str(args.port)], env=env)
    try:
        if not wait_port(args.port):
            print("FAIL: sim_bridge did not start", file=sys.stderr)
            return 1
        print(f"--- brain_pi neutral_hold for {args.duration:.0f}s against sim_bridge ---", flush=True)
        proc = subprocess.run(
            [sys.executable, "-m", "cli", "run", "--mode", "neutral_hold", "--link", f"tcp://127.0.0.1:{args.port}",
             "--duration", str(args.duration)],
            env=env, capture_output=True, text=True, timeout=args.duration + 30)
        sys.stdout.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("SUMMARY ")), None)
        if proc.returncode != 0 or line is None:
            print("FAIL: runner exited abnormally", file=sys.stderr)
            return 1
        r = json.loads(line[8:])
        checks = {
            f"action rate ~50 Hz (got {r['tx_hz']})": 45 <= r["tx_hz"] <= 55,
            f"state rate >= {args.min_state_hz:.0f} Hz (got {r['rx_hz']})": r["rx_hz"] >= args.min_state_hz,
            f"no CRC/length errors (crc={r['crc_errors']} len={r['len_errors']})": r["crc_errors"] == 0 and r["len_errors"] == 0,
            f"ESP acked recent action (sent {r['last_sent_seq']}, acked {r['last_ack_seq']})":
                r["last_ack_seq"] is not None and 0 <= r["last_sent_seq"] - r["last_ack_seq"] <= 3,
        }
        ok = True
        for name, passed in checks.items():
            print(("PASS  " if passed else "FAIL  ") + name)
            ok &= passed
        print("BENCH TEST " + ("PASSED" if ok else "FAILED"))
        return 0 if ok else 1
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=3)
        except subprocess.TimeoutExpired:
            bridge.kill()


if __name__ == "__main__":
    sys.exit(main())
