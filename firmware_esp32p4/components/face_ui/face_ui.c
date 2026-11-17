#include "face_ui.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "robot_protocol.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "face_ui";
#define FRAME_MS 16
#define N_RIPPLES 4

typedef struct { lv_obj_t *sclera, *iris, *pupil, *glint, *lid; float cx, cy, d; } eye_t;
typedef struct { lv_obj_t *ring; float age, intensity; bool live; } ripple_t;

static eye_t s_eye[2];
static ripple_t s_rip[N_RIPPLES];
static int s_size, s_cx, s_cy;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static face_ui_inputs_t s_in;

/* animation state */
static float s_ex, s_ey, s_evx, s_evy;       /* eye offset (normalised) and velocity */
static float s_arousal, s_lid;               /* smoothed */
static float s_blink_t = -1.0f, s_next_blink = 3.0f, s_t, s_next_micro = 1.0f, s_mx, s_my;
static float s_load_ema;

static lv_obj_t *circle(lv_obj_t *parent, int d, uint32_t rgb) {
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, d, d);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(rgb), 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static void centre(lv_obj_t *o, float cx, float cy) {
    lv_obj_set_pos(o, (int)lroundf(cx - lv_obj_get_width(o) * 0.5f), (int)lroundf(cy - lv_obj_get_height(o) * 0.5f));
}

static float frand(void) { return (float)rand() / (float)RAND_MAX; }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void spawn_ripple(float intensity) {
    for (int i = 0; i < N_RIPPLES; i++) {
        if (!s_rip[i].live) { s_rip[i].live = true; s_rip[i].age = 0; s_rip[i].intensity = clampf(intensity, 0.3f, 1.0f); return; }
    }
}

static void frame_cb(lv_timer_t *t) {
    (void)t;
    const float dt = FRAME_MS / 1000.0f;
    face_ui_inputs_t in;
    portENTER_CRITICAL(&s_lock);
    in = s_in;
    portEXIT_CRITICAL(&s_lock);
    s_t += dt;

    /* --- eye saccade: stiff, slightly under-damped spring toward the gaze error --- */
    if (s_t >= s_next_micro) { s_mx = (frand() - 0.5f) * 0.10f; s_my = (frand() - 0.5f) * 0.08f; s_next_micro = s_t + 0.4f + frand() * 1.6f; }
    float w = in.gaze_conf > 0.3f ? 1.0f : 0.0f; /* only follow targets we actually believe in */
    float tx = clampf(in.gaze_x * w + s_mx, -1, 1), ty = clampf(in.gaze_y * w + s_my, -1, 1);
    const float k = 1400.0f, c = 2.0f * sqrtf(k) * 0.75f;
    s_evx += (k * (tx - s_ex) - c * s_evx) * dt;
    s_evy += (k * (ty - s_ey) - c * s_evy) * dt;
    s_ex += s_evx * dt;
    s_ey += s_evy * dt;

    /* --- arousal -> pupil size; flinch dilates hard and squints --- */
    bool flinch = in.reflex_mask & ROBOT_REFLEX_BIT_LOOMING_FLINCH;
    float a_t = 0.15f + 0.45f * in.gaze_conf * (in.gaze_locked ? 1.0f : 0.6f) + (flinch ? 0.4f : 0.0f);
    s_arousal += (clampf(a_t, 0, 1) - s_arousal) * clampf(dt * (flinch ? 30.0f : 4.0f), 0, 1);

    /* --- eyelids: blink + flinch squint --- */
    float lid_t = flinch ? 0.45f : 0.0f;
    if (s_blink_t < 0 && s_t >= s_next_blink) { s_blink_t = 0; }
    if (s_blink_t >= 0) {
        s_blink_t += dt;
        float ph = s_blink_t / 0.16f;  /* close+open in 160 ms */
        lid_t = fmaxf(lid_t, ph < 0.5f ? ph * 2.0f : (ph < 1.0f ? (1.0f - ph) * 2.0f : 0.0f));
        if (ph >= 1.0f) { s_blink_t = -1; s_next_blink = s_t + 2.5f + frand() * 3.5f; }
    }
    s_lid += (lid_t - s_lid) * clampf(dt * 25.0f, 0, 1);

    /* --- footstep ripples from ankle load spikes --- */
    float spike = in.foot_load - s_load_ema;
    s_load_ema += (in.foot_load - s_load_ema) * 0.05f;
    if (spike > 0.15f) spawn_ripple(spike * 2.0f);

    /* --- apply to LVGL objects --- */
    for (int e = 0; e < 2; e++) {
        eye_t *y = &s_eye[e];
        float travel = y->d * 0.16f;
        float ix = y->cx + s_ex * travel, iy = y->cy + s_ey * travel;
        float pupil_d = lv_obj_get_width(y->iris) * (0.35f + 0.40f * s_arousal);
        lv_obj_set_size(y->pupil, (int)pupil_d, (int)pupil_d);
        centre(y->iris, ix, iy);
        centre(y->pupil, ix, iy);
        centre(y->glint, ix - y->d * 0.07f, iy - y->d * 0.07f);
        int lid_h = (int)(s_lid * y->d);
        lv_obj_set_height(y->lid, lid_h > 0 ? lid_h : 1);
        lv_obj_set_style_bg_opa(y->lid, lid_h > 0 ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    }
    for (int i = 0; i < N_RIPPLES; i++) {
        ripple_t *r = &s_rip[i];
        if (!r->live) { lv_obj_add_flag(r->ring, LV_OBJ_FLAG_HIDDEN); continue; }
        r->age += dt;
        float p = r->age / 0.7f;
        if (p >= 1.0f) { r->live = false; continue; }
        int d = (int)(s_size * (0.35f + 0.65f * p));
        lv_obj_remove_flag(r->ring, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(r->ring, d, d);
        centre(r->ring, s_cx, s_cy);
        lv_obj_set_style_border_opa(r->ring, (lv_opa_t)(255.0f * r->intensity * (1.0f - p)), 0);
    }
}

esp_err_t face_ui_start(lv_display_t *disp) {
    if (!disp) return ESP_ERR_INVALID_ARG;
    int w = lv_display_get_horizontal_resolution(disp), h = lv_display_get_vertical_resolution(disp);
    s_size = w < h ? w : h;
    s_cx = w / 2;
    s_cy = h / 2;
    if (!lvgl_port_lock(1000)) return ESP_ERR_TIMEOUT;

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x05070A), 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < N_RIPPLES; i++) {  /* rings first => drawn behind the eyes */
        lv_obj_t *o = circle(scr, s_size / 2, 0x000000);
        lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(o, 4, 0);
        lv_obj_set_style_border_color(o, lv_color_hex(0x4FC3F7), 0);
        lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
        s_rip[i].ring = o;
    }
    const float d = s_size * 0.34f;
    for (int e = 0; e < 2; e++) {
        eye_t *y = &s_eye[e];
        y->d = d;
        y->cx = s_cx + (e ? 1 : -1) * s_size * 0.19f;
        y->cy = s_cy - s_size * 0.02f;
        y->sclera = circle(scr, (int)d, 0xF2F5F7);
        centre(y->sclera, y->cx, y->cy);
        y->iris = circle(scr, (int)(d * 0.58f), 0x2E86DE);
        y->pupil = circle(scr, (int)(d * 0.58f * 0.5f), 0x050505);
        y->glint = circle(scr, (int)(d * 0.09f), 0xFFFFFF);
        y->lid = lv_obj_create(scr);  /* eyelid: background-coloured rectangle sliding down over the eye */
        lv_obj_remove_style_all(y->lid);
        lv_obj_set_size(y->lid, (int)d + 2, 1);
        lv_obj_set_style_bg_color(y->lid, lv_color_hex(0x05070A), 0);
        lv_obj_set_style_bg_opa(y->lid, LV_OPA_TRANSP, 0);
        lv_obj_set_pos(y->lid, (int)(y->cx - d * 0.5f) - 1, (int)(y->cy - d * 0.5f));
        lv_obj_remove_flag(y->lid, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    }
    lv_timer_create(frame_cb, FRAME_MS, NULL);
    lvgl_port_unlock();
    ESP_LOGI(TAG, "face UI running on %dx%d display @ %d FPS target", w, h, 1000 / FRAME_MS);
    return ESP_OK;
}

void face_ui_update(const face_ui_inputs_t *in) {
    portENTER_CRITICAL(&s_lock);
    s_in = *in;
    portEXIT_CRITICAL(&s_lock);
}
