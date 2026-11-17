/*
 * Motion-saliency tracker: |frame - previous frame| thresholded, centroid of the changed pixels.
 * Looming = the changed area grows quickly while the neck is (nearly) still. Known limits:
 * ego-motion from the robot's own body sway produces global change (looming is suppressed while
 * the neck moves fast), and a static target is invisible -- swap in an ESP-DL detector for that.
 */
#include "foveated_vision.h"
#include <math.h>
#include <string.h>

#define W 80
#define H 60
#define DIFF_THRESH 18
#define MIN_PIXELS 12
#define FULL_CONF_FRAC 0.02f   /* 2% of the frame changed => confidence 1.0 */
#define LOOM_MIN_FRAC 0.06f
#define LOOM_GROWTH 1.5f       /* area now vs 3 frames ago */
#define LOOM_NECK_SPEED_MAX 1.0f
#define LOOM_COOLDOWN_FRAMES 10
#define HIST 4

static uint8_t s_prev[W * H];
static bool s_have_prev;
static float s_area_hist[HIST];
static int s_cooldown;
static float s_x, s_y;

static bool process(const uint8_t *luma, int w, int h, float neck_speed, fovea_gaze_t *out, bool *looming) {
    *looming = false;
    if (w != W || h != H) return false;
    if (!s_have_prev) { memcpy(s_prev, luma, sizeof(s_prev)); s_have_prev = true; return false; }

    int count = 0;
    long sx = 0, sy = 0;
    for (int y = 1; y < H - 1; y++) {
        for (int x = 1; x < W - 1; x++) {
            int d = (int)luma[y * W + x] - (int)s_prev[y * W + x];
            if (d > DIFF_THRESH || d < -DIFF_THRESH) { count++; sx += x; sy += y; }
        }
    }
    memcpy(s_prev, luma, sizeof(s_prev));

    float frac = (float)count / (float)(W * H);
    for (int i = HIST - 1; i > 0; i--) s_area_hist[i] = s_area_hist[i - 1];
    s_area_hist[0] = frac;
    if (s_cooldown > 0) s_cooldown--;
    if (frac > LOOM_MIN_FRAC && s_area_hist[HIST - 1] > 0.0005f && frac > LOOM_GROWTH * s_area_hist[HIST - 1] &&
        fabsf(neck_speed) < LOOM_NECK_SPEED_MAX && s_cooldown == 0) {
        *looming = true;
        s_cooldown = LOOM_COOLDOWN_FRAMES;
    }

    if (count < MIN_PIXELS) { out->confidence *= 0.8f; if (out->confidence < 0.05f) out->confidence = 0.0f; return false; }
    float cx = ((float)sx / count) / (W - 1) * 2.0f - 1.0f;
    float cy = ((float)sy / count) / (H - 1) * 2.0f - 1.0f;
    s_x += 0.6f * (cx - s_x);  /* light smoothing */
    s_y += 0.6f * (cy - s_y);
    out->x = s_x;
    out->y = s_y;
    out->confidence = fminf(1.0f, frac / FULL_CONF_FRAC);
    return true;
}

const fovea_tracker_ops_t fovea_motion_tracker = {.process = process};
