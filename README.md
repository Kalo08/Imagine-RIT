# Dual-brain desktop biped

ESP32-P4 ("spinal cord & visual cortex") + Raspberry Pi ("onboard runtime") for an original
2-legged desktop robot. The joint count **N is not hard-coded anywhere**: it comes from
[`shared/robot_spec.yaml`](shared/robot_spec.yaml).

```
shared/                 robot_spec.yaml  protocol.yaml  generate_bindings.py   <- single source of truth
firmware_esp32p4/       ESP-IDF project (C): servo_bus, imu_bno085, spinal_reflex, foveated_vision, face_ui, usb_bridge
brain_pi/               Python 3.12 runtime: 50 Hz policy_runner, usb_client, fovea_agent, cli, policies/
tools/                  sim_bridge.py (fake ESP32-P4), bench_test.py, servo_zero_calibrator.py
```

## Quick start (no hardware)

```bash
python shared/generate_bindings.py        # or: make generate
python tools/bench_test.py                # or: make bench-test
```

`bench-test` starts `sim_bridge.py` (200 Hz `StatePacket`s, synthetic IMU, servo echo, gaze sweep),
runs `brain_pi` in `neutral_hold` for 5 s and checks: ~50 Hz actions out, ≥150 Hz states in,
zero CRC/length errors, and that the sim acknowledged the latest `sequence_id`.
Windows has no `make` by default; every Makefile target is a one-line `python` command (see `Makefile`).

Pi install: `cd brain_pi && uv sync` (add `--extra onnx` to run policies). Then e.g.
`python -m cli run --mode neutral_hold --link serial:/dev/ttyACM0` (run with `PYTHONPATH=brain_pi/src`,
or use the installed `robot-brain` command). Useful: `cli jog`, `cli monitor`, `cli estop`, `cli info`.

## Changing the joint list

1. Edit `joints:` in `shared/robot_spec.yaml` (and joint names in `reflexes.*.offsets_rad` if used).
2. `make generate` → rewrites `firmware_esp32p4/components/generated/include/robot_protocol.h` and
   `brain_pi/src/generated/robot_protocol.py`: `ROBOT_NUM_JOINTS`, packed structs with `static_assert`ed sizes,
   per-joint limit tables, reflex offset tables, packet sizes.
3. Rebuild/reflash the ESP32-P4. The Pi refuses to start if the generated code is stale vs the YAML
   (`cli` compares a spec hash), and both sides reject frames whose length doesn't match their build.
4. Policies trained for the old joint order are rejected by the manifest check.

## Protocol (details in `shared/protocol.yaml`)

Frame: `5A A5 | type u8 | len u16 | payload | CRC-16/CCITT-FALSE u16`, little-endian, packed.
`StatePacket` (ESP→Pi, 200 Hz, 116 B for N=6), `ActionPacket` (Pi→ESP, 50 Hz, 30 B for N=6),
`RoiFrame` (ESP→Pi, ≤10 Hz, 64×64 gray crop while gaze is locked). Control modes: `NEUTRAL_HOLD`,
`ONNX_POLICY` (= "follow the Pi's targets": used for both policy output and manual jog), `ESTOP`
(torque off, latched until the Pi sends `NEUTRAL_HOLD`).

## Safety model

* ESP32-P4 Core 1 (`spinal_reflex`, pure C) is the last line of defence and cannot be bypassed by the Pi:
  NaN/limit clamping → slew limit → reflex offsets → over-current clamp → hard `[min,max]` clamp.
* Pi silent for `action_timeout_ms` → hold neutral. `ESTOP` is latched; a dead link never re-enables torque.
* The Pi additionally clamps/slew-limits, falls back to `NEUTRAL_HOLD` on stale telemetry or any policy error,
  and only enables `onnx` mode after `manifest.json` matches the robot spec exactly
  (see [`brain_pi/policies/README.md`](brain_pi/policies/README.md)).

## Status — read this before powering servos

| Part | Status |
|---|---|
| `shared/` generator, Python bindings, `brain_pi`, `sim_bridge`, `bench_test`, calibrator | **Run and passing** on this machine (bench test: 200 Hz / 50 Hz, 0 CRC errors) |
| `firmware_esp32p4/` | **Written but never compiled or run** (no ESP-IDF/toolchain here). Expect compile fixes and API drift (esp_video, esp_tinyusb, esp_lvgl_port). Pin numbers in the `Kconfig` files are placeholders. See [`firmware_esp32p4/README.md`](firmware_esp32p4/README.md) |
| ONNX path | Contract/validation implemented; **not exercised** (no model, `onnxruntime` not installed here) |

No RL training, simulation, or model-export code exists in this repo by design.
