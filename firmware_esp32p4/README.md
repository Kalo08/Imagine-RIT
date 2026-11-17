# firmware_esp32p4

ESP-IDF ≥ 5.4, target `esp32p4`. Build: `make firmware-build` (or `idf.py set-target esp32p4 && idf.py build`) from the repo root / this folder.
Always run `make generate` first; `robot_protocol.h` is generated.

| Component | Role |
|---|---|
| `generated` | header-only `robot_protocol.h` (structs, CRC16, resyncing parser, per-joint tables) |
| `servo_bus` | `servo_driver_t` abstraction; Feetech STS and Dynamixel X (Protocol 2.0) drivers; chosen by `bus.driver` in the spec |
| `imu_bno085` | BNO085 over I2C/SHTP: game rotation vector + gyro @ 4 ms |
| `spinal_reflex` | hard safety + reflexes; pure C, host-testable, no ESP-IDF includes |
| `foveated_vision` | CSI camera → motion-saliency tracker → neck PID mailbox, looming flag, 64×64 ROI |
| `face_ui` | LVGL 9 eyes: saccade spring, pupil dilation, blink, flinch squint, footstep ripples |
| `usb_bridge` | TinyUSB HS CDC-ACM, CRC-verified RX → action mailbox, TX queue (drops oldest) |

Core split: **Core 1** = `control_task` (200 Hz, sole owner of the servo UART). **Core 0** = vision, USB, LVGL.
Deviation from the brief: the neck PID runs on Core 0 but its *output goes through a mailbox* to the Core 1 loop
rather than driving the bus directly, so only one task ever touches the half-duplex UART.

## Before first power-up

1. Set pins in `idf.py menuconfig` (Servo bus, BNO085, Camera SCCB) — defaults are placeholders.
2. Check servo control-table constants in `servo_feetech.c` / `servo_dynamixel.c` against your exact model/firmware.
3. Bench with **servos unpowered or horns removed**; use `tools/servo_zero_calibrator.py`, then `cli jog` at small offsets.
4. Verify `max_current_ma`, limits and `max_vel_rad_s` in `robot_spec.yaml` for your mechanics.

## Not implemented / known gaps

* **Never compiled** — see root README status.
* **H.264 streaming**: not implemented. The Pi receives a 64×64 gray fovea crop (`RoiFrame`); an `esp_h264` encoder would need its own packet type.
* **2D PPA** offload for the face UI is not wired; the UI uses plain LVGL draw. Add via LVGL/BSP PPA draw-unit config if 60 FPS is not met.
* **ESP-DL** detector: not included; implement `fovea_tracker_ops_t` and pass it to `foveated_vision_start()`. The default tracker sees only moving things and can mistake ego-motion for targets.
* **Display bring-up** is the single seam in `main/board_display.c` (uses the board BSP if present).
* BNO085 over I2C; SPI would give more timing margin at 200 Hz.
* No on-target or host unit tests yet for `spinal_reflex` (it is written to be host-compilable).
