#include "foveated_vision.h"
#include "camera_v4l2.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "fovea";
#define TW 80
#define TH 60
#define ROI_STRIDE 2        /* source pixels per ROI pixel: 64*2 = 128 px window at 640x480 */
#define ROI_PERIOD_US 100000

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static fovea_gaze_t s_gaze;
static float s_neck_target[ROBOT_NUM_JOINTS];
static bool s_neck_valid;
static float s_fb_pos[ROBOT_NUM_JOINTS], s_fb_vel[ROBOT_NUM_JOINTS];
static volatile bool s_looming;
static fovea_roi_sink_t s_sink;
static const fovea_tracker_ops_t *s_tracker;

void foveated_vision_set_roi_sink(fovea_roi_sink_t sink) { s_sink = sink; }

void foveated_vision_update_neck_feedback(const float pos[ROBOT_NUM_JOINTS], const float vel[ROBOT_NUM_JOINTS]) {
    portENTER_CRITICAL(&s_lock);
    memcpy(s_fb_pos, pos, sizeof(s_fb_pos));
    memcpy(s_fb_vel, vel, sizeof(s_fb_vel));
    portEXIT_CRITICAL(&s_lock);
}

bool foveated_vision_get_neck_targets(float targets[ROBOT_NUM_JOINTS]) {
    bool ok;
    portENTER_CRITICAL(&s_lock);
    ok = s_neck_valid;
    if (ok) for (int i = 0; i < ROBOT_NUM_JOINTS; i++) if (ROBOT_JOINT_IS_NECK[i]) targets[i] = s_neck_target[i];
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

bool foveated_vision_take_looming(void) {
    if (!s_looming) return false;
    s_looming = false;
    return true;
}

void foveated_vision_get_gaze(fovea_gaze_t *g) {
    portENTER_CRITICAL(&s_lock);
    *g = s_gaze;
    portEXIT_CRITICAL(&s_lock);
}

/* RGB565 -> 8-bit luma (integer Rec.601 approximation). */
static inline uint8_t luma565(uint16_t p) {
    uint32_t r = (p >> 11) & 31, g = (p >> 5) & 63, b = p & 31;
    return (uint8_t)((r * 8 * 77 + g * 4 * 150 + b * 8 * 29) >> 8);
}

static void downsample(const uint16_t *f, uint8_t *out) {
    for (int y = 0; y < TH; y++)
        for (int x = 0; x < TW; x++) out[y * TW + x] = luma565(f[(y * (CAM_H / TH)) * CAM_W + x * (CAM_W / TW)]);
}

static void make_roi(const uint16_t *f, float gx, float gy, robot_roi_frame_t *roi) {
    int cx = (int)((gx * 0.5f + 0.5f) * CAM_W), cy = (int)((gy * 0.5f + 0.5f) * CAM_H);
    int x0 = cx - (ROBOT_ROI_W * ROI_STRIDE) / 2, y0 = cy - (ROBOT_ROI_H * ROI_STRIDE) / 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 > CAM_W - ROBOT_ROI_W * ROI_STRIDE) x0 = CAM_W - ROBOT_ROI_W * ROI_STRIDE;
    if (y0 > CAM_H - ROBOT_ROI_H * ROI_STRIDE) y0 = CAM_H - ROBOT_ROI_H * ROI_STRIDE;
    for (int y = 0; y < ROBOT_ROI_H; y++)
        for (int x = 0; x < ROBOT_ROI_W; x++)
            roi->pixels[y * ROBOT_ROI_W + x] = luma565(f[(y0 + y * ROI_STRIDE) * CAM_W + x0 + x * ROI_STRIDE]);
    roi->timestamp_us = (uint64_t)esp_timer_get_time();
    roi->center_x = gx;
    roi->center_y = gy;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void vision_task(void *arg) {
    (void)arg;
    static uint8_t luma[TW * TH];
    static robot_roi_frame_t roi;
    fovea_gaze_t g = {0};
    float integ[ROBOT_NUM_JOINTS] = {0}, prev_err[ROBOT_NUM_JOINTS] = {0};
    float tgt[ROBOT_NUM_JOINTS];
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) tgt[i] = ROBOT_JOINT_NEUTRAL_RAD[i];
    int locked_frames = 0;
    int64_t last_t = esp_timer_get_time(), last_roi = 0;

    for (;;) {
        const uint16_t *frame = camera_grab();
        if (!frame) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        int64_t now = esp_timer_get_time();
        float dt = clampf((float)(now - last_t) * 1e-6f, 0.005f, 0.2f);
        last_t = now;

        downsample(frame, luma);
        float pos[ROBOT_NUM_JOINTS], vel[ROBOT_NUM_JOINTS];
        portENTER_CRITICAL(&s_lock);
        memcpy(pos, s_fb_pos, sizeof(pos));
        memcpy(vel, s_fb_vel, sizeof(vel));
        portEXIT_CRITICAL(&s_lock);
        float neck_speed = 0.0f;
        for (int i = 0; i < ROBOT_NUM_JOINTS; i++) if (ROBOT_JOINT_IS_NECK[i]) neck_speed = fmaxf(neck_speed, fabsf(vel[i]));

        bool looming = false;
        bool have = s_tracker->process(luma, TW, TH, neck_speed, &g, &looming);
        if (looming) s_looming = true;

        /* --- neck PID: image error -> joint-angle error -> incremental target --- */
        bool engaged = have && g.confidence >= ROBOT_GAZE_MIN_CONFIDENCE;
        float err_img[3] = {0, g.x, g.y}; /* index by ROBOT_JOINT_GAZE_AXIS: 1 = yaw(x), 2 = pitch(y) */
        float max_err = 0.0f;
        for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
            int axis = ROBOT_JOINT_GAZE_AXIS[i];
            if (!axis) continue;
            if (!engaged) { integ[i] = 0; prev_err[i] = 0; continue; }
            float fov = (axis == 1) ? ROBOT_GAZE_HFOV_RAD : ROBOT_GAZE_VFOV_RAD;
            float sign = (axis == 1) ? ROBOT_GAZE_YAW_SIGN : ROBOT_GAZE_PITCH_SIGN;
            float err = sign * err_img[axis] * 0.5f * fov;  /* radians the neck should still turn */
            integ[i] = clampf(integ[i] + err * dt, -0.5f, 0.5f);
            float d = (err - prev_err[i]) / dt;
            prev_err[i] = err;
            tgt[i] = clampf(pos[i] + ROBOT_GAZE_KP * err + ROBOT_GAZE_KI * integ[i] + ROBOT_GAZE_KD * d,
                            ROBOT_JOINT_MIN_RAD[i], ROBOT_JOINT_MAX_RAD[i]);
            max_err = fmaxf(max_err, fabsf(err_img[axis]));
        }
        if (engaged && max_err < ROBOT_GAZE_LOCK_THRESHOLD) locked_frames++; else locked_frames = 0;
        g.locked = locked_frames >= ROBOT_GAZE_LOCK_FRAMES;

        portENTER_CRITICAL(&s_lock);
        s_gaze = g;
        memcpy(s_neck_target, tgt, sizeof(tgt));
        s_neck_valid = true;
        portEXIT_CRITICAL(&s_lock);

        if (s_sink && g.locked && now - last_roi >= ROI_PERIOD_US) {
            make_roi(frame, g.x, g.y, &roi);
            s_sink(&roi);
            last_roi = now;
        }
        camera_release();
    }
}

esp_err_t foveated_vision_start(int core, const fovea_tracker_ops_t *tracker) {
    s_tracker = tracker ? tracker : &fovea_motion_tracker;
    esp_err_t e = camera_open();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "camera unavailable (%s): neck stays under Pi/neutral control", esp_err_to_name(e));
        return e;
    }
    xTaskCreatePinnedToCore(vision_task, "vision", 8192, NULL, 8, NULL, core);
    return ESP_OK;
}
