/*
 * foveated_vision -- Core-0 gaze tracker driving the neck joints.
 *
 *   MIPI-CSI camera (V4L2 via esp_video) -> 80x60 luma -> tracker -> (x, y, confidence)
 *        -> PID -> neck joint targets  --mailbox-->  Core-1 control loop (single owner of the servo UART)
 *        -> looming detector           --flag----->  spinal_reflex (flinch)
 *        -> 64x64 foveal crop @10 Hz   --sink----->  usb_bridge (to the Pi)
 *
 * The neck PID runs here on Core 0 but the *servo writes* stay on Core 1: exactly one task ever
 * touches the half-duplex bus, which removes bus-contention hazards. The target mailbox adds
 * <= 1 control tick (5 ms) of latency on top of the ~33 ms camera frame.
 *
 * The default tracker is motion saliency (frame differencing + centroid). It is intentionally
 * behind fovea_tracker_ops_t so an ESP-DL detector (face / person) can replace it.
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "robot_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y;        /* normalised image coords, -1..1 (x right, y down) */
    float confidence;  /* 0..1 */
    bool locked;
} fovea_gaze_t;

/* Pluggable tracker: consume an 80x60 gray frame, report the best target. */
typedef struct {
    bool (*process)(const uint8_t *luma, int w, int h, float neck_speed_rad_s, fovea_gaze_t *out, bool *looming);
} fovea_tracker_ops_t;

typedef void (*fovea_roi_sink_t)(const robot_roi_frame_t *roi);

esp_err_t foveated_vision_start(int core, const fovea_tracker_ops_t *tracker /* NULL = motion saliency */);
void foveated_vision_set_roi_sink(fovea_roi_sink_t sink);

/* Core 1 -> Core 0: current neck feedback so the PID can work in absolute joint angles. */
void foveated_vision_update_neck_feedback(const float pos_rad[ROBOT_NUM_JOINTS], const float vel_rad_s[ROBOT_NUM_JOINTS]);
/* Core 0 -> Core 1: latest neck targets (entries for non-neck joints are left untouched). */
bool foveated_vision_get_neck_targets(float targets[ROBOT_NUM_JOINTS]);
/* One-shot looming flag; returns true once per detection. */
bool foveated_vision_take_looming(void);
void foveated_vision_get_gaze(fovea_gaze_t *g);

/* Built-in motion-saliency tracker (exposed for host-side testing / composition). */
extern const fovea_tracker_ops_t fovea_motion_tracker;

#ifdef __cplusplus
}
#endif
