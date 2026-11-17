/*
 * face_ui -- force-coupled eye/face renderer (LVGL 9) for the round MIPI-DSI display. Core 0.
 *
 * Behaviours:
 *   * Eye saccades are driven by the *gaze error* (where the target is), not by neck position,
 *     so the eyes snap to a new target ahead of the slower neck -- like a real saccade.
 *   * Pupil dilation follows arousal: tracking confidence / lock, and a sudden dilation on flinch.
 *   * Footstep impacts (ankle load spikes) spawn expanding ripple rings.
 *   * Blinks, idle micro-saccades, a squint while a flinch reflex is active.
 *
 * Rendering uses LVGL primitives (circles) only; the display flush path is the board BSP's.
 * PPA (2D accelerator) offload of LVGL blending is a display-driver/LVGL-config matter and is
 * not wired in this repository -- see firmware README "Not implemented / next steps".
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gaze_x, gaze_y;   /* -1..1 */
    float gaze_conf;        /* 0..1 */
    bool gaze_locked;
    uint16_t reflex_mask;   /* ROBOT_REFLEX_BIT_* */
    float foot_load;        /* 0..1: max |ankle current| / limit; spikes => footstep ripple */
} face_ui_inputs_t;

/* Build the scene on `disp` (from the board's display init) and start the 60 FPS animation timer. */
esp_err_t face_ui_start(lv_display_t *disp);
/* Thread-safe: latest inputs, consumed by the next animation frame. */
void face_ui_update(const face_ui_inputs_t *in);

#ifdef __cplusplus
}
#endif
