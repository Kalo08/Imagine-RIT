# Policy bundles — contract for the RL teammate

Drop a bundle here and the Pi runtime picks it up. **Nothing in this repo trains or
exports policies**; this is only the interface.

```
brain_pi/policies/
  my_policy_v1/            <- any directory name (newest manifest.json wins), or put the two files directly in policies/
    policy.onnx
    manifest.json
```

`make validate-policy` (or `python -m cli validate-policy`) checks a bundle against
`shared/robot_spec.yaml` without touching hardware. `robot-brain run --mode onnx|auto`
runs the same checks and **refuses to enable motor output on any mismatch**.
With `--mode auto` and no (valid) bundle the runtime falls back to `neutral_hold`.

## The robot you are targeting

`NUM_JOINTS`, joint order, limits and neutral pose come from `shared/robot_spec.yaml`
and **can change while the CAD is finalised**. Run `python -m cli info` for the current
table. Your manifest must list the exact `joint_names` in that order; if the robot spec
changes, you re-export and the old bundle is rejected.

## ONNX model contract

* Stateless feed-forward model (no recurrent state tensors). Use the `last_action`
  observation term if you need action history.
* Input 0: `float32`, shape `[1, obs_dim]` (batch dimension 1 or dynamic).
* Output 0: `float32`, shape `[1, NUM_JOINTS]`.
* Runs at exactly `control_hz = 50` (the robot's `action_hz`).
* Inference runs on the Pi CPU (onnxruntime, CPUExecutionProvider).

## manifest.json

```json
{
  "schema_version": 1,
  "policy_name": "walk_v3",
  "onnx_file": "policy.onnx",
  "onnx_sha256": "optional hex digest of policy.onnx",
  "spec_hash": "optional: value of `python -m cli info` spec_hash used at training time",
  "num_joints": 6,
  "joint_names": ["neck_yaw", "neck_pitch", "l_hip_pitch", "r_hip_pitch", "l_ankle_pitch", "r_ankle_pitch"],
  "control_hz": 50,
  "observation": {
    "input_name": "obs",
    "dim": 22,
    "terms": [
      {"name": "joint_pos_rel",       "scale": 1.0},
      {"name": "joint_vel",           "scale": 0.1},
      {"name": "imu_gyro",            "scale": 0.25},
      {"name": "projected_gravity"},
      {"name": "command", "size": 1}
    ]
  },
  "action": {
    "output_name": "action",
    "mode": "delta_from_neutral",
    "scale": 0.5,
    "clip": 1.0
  }
}
```

### Rules the loader enforces

| Field | Rule |
|---|---|
| `schema_version` | must be `1` |
| `num_joints` | must equal `NUM_JOINTS` in `robot_spec.yaml` |
| `joint_names` | must equal the spec's joint names **in the same order** |
| `control_hz` | must equal the robot's `action_hz` (50) |
| `spec_hash` | optional; if present must equal the current spec hash |
| `onnx_sha256` | optional; if present must match `policy.onnx` |
| `observation.dim` | must equal the sum of all term sizes **and** the ONNX input width |
| `observation.input_name` / `action.output_name` | must match the ONNX graph names |
| ONNX output width | must equal `NUM_JOINTS` |

### Observation terms

Terms are concatenated **in the order listed**, each multiplied by its `scale` (default 1).

| `name` | size | meaning |
|---|---|---|
| `joint_pos_rel` | N | measured position − `neutral_rad` (rad) |
| `joint_vel` | N | measured velocity (rad/s) |
| `joint_current_norm` | N | `current_ma / max_current_ma` |
| `last_action` | N | previous **raw** network output (zeros on the first tick) |
| `imu_quat_wxyz` | 4 | orientation quaternion (w, x, y, z) |
| `imu_gyro` | 3 | body-frame angular rate (rad/s) |
| `projected_gravity` | 3 | unit gravity direction in the body frame (upright ≈ `[0, 0, -1]`) |
| `gaze` | 3 | foveated target `x`, `y` (−1..1) and `locked` (0/1) |
| `command` | `size` (you set it) | high-level command vector; zeros until a command source is wired up |

### Action interpretation

`raw = clip(output, ±clip)` if `clip` is given, then

* `delta_from_neutral`: `target_rad = neutral_rad + raw * scale`
* `absolute_rad`: `target_rad = raw * scale`

The runtime then clamps to `[min_rad, max_rad]`, slew-limits to `max_vel_rad_s`, and the
ESP32-P4 applies its own hard limits again. NaN/Inf output or an exception latches
`NEUTRAL_HOLD`.

Neck joints keep tracking gaze on the ESP32-P4 unless the Pi runs with `--gaze-override`
(then your policy's neck outputs are used).
