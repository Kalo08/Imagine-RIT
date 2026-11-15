#include "spinal_reflex.h"
#include <math.h>

#define N ROBOT_NUM_JOINTS

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* 1.0 during hold, linear ramp to 0 over release, 0 afterwards. */
static float envelope(bool active, uint64_t now, uint64_t t0, float hold_ms, float release_ms) {
    if (!active) return 0.0f;
    float el = (float)(now - t0) * 1e-3f;
    if (el <= hold_ms) return 1.0f;
    if (el >= hold_ms + release_ms) return 0.0f;
    return 1.0f - (el - hold_ms) / release_ms;
}

void spinal_reflex_init(spinal_ctx_t *c) { memset(c, 0, sizeof(*c)); }

void spinal_reflex_step(spinal_ctx_t *c, const spinal_in_t *in, spinal_out_t *out) {
    uint16_t mask = 0;

    /* ---- time base ------------------------------------------------------------ */
    float dt = c->initialised ? (float)(in->now_us - c->last_us) * 1e-6f : 1.0f / ROBOT_STATE_HZ;
    dt = clampf(dt, 0.0005f, 0.020f);
    c->last_us = in->now_us;

    /* ---- mode arbitration -------------------------------------------------------
     * ESTOP is latched. It is cleared ONLY by a live Pi sending NEUTRAL_HOLD (explicit
     * re-arm); a dead link never re-enables torque. A silent Pi (not latched) => hold neutral. */
    if (in->pi_mode == ROBOT_CONTROL_MODE_ESTOP) c->estop_latched = true;
    else if (in->pi_link_alive && in->pi_mode == ROBOT_CONTROL_MODE_NEUTRAL_HOLD) c->estop_latched = false;

    robot_control_mode_t mode = in->pi_mode;
    if (!in->pi_link_alive) { mode = ROBOT_CONTROL_MODE_NEUTRAL_HOLD; mask |= ROBOT_REFLEX_BIT_WATCHDOG_HOLD; }
    if (c->estop_latched) mode = ROBOT_CONTROL_MODE_ESTOP;
    if (mode != ROBOT_CONTROL_MODE_NEUTRAL_HOLD && mode != ROBOT_CONTROL_MODE_ONNX_POLICY &&
        mode != ROBOT_CONTROL_MODE_ESTOP) mode = ROBOT_CONTROL_MODE_NEUTRAL_HOLD; /* unknown value => safe */

    out->active_mode = mode;
    if (!in->imu_valid) mask |= ROBOT_REFLEX_BIT_IMU_STALE;
    if (in->servo_fault) mask |= ROBOT_REFLEX_BIT_SERVO_FAULT;

    if (mode == ROBOT_CONTROL_MODE_ESTOP) {
        /* torque off: servos are limp, so "target" is just where they currently are */
        for (int i = 0; i < N; i++) { out->target[i] = in->pos[i]; c->cmd[i] = in->pos[i]; c->prev_abs_cur[i] = 0; c->oc_accum_us[i] = 0; c->oc_clamped[i] = false; }
        c->flinch_active = c->brace_active = false;
        c->torque_prev = false;
        c->initialised = true;
        out->torque_enable = false;
        out->reflex_mask = mask;
        return;
    }

    /* torque just (re)enabled: start the slew from where the joints are -> no lurch */
    if (!c->torque_prev || !c->initialised) {
        for (int i = 0; i < N; i++) { c->cmd[i] = clampf(in->pos[i], ROBOT_JOINT_MIN_RAD[i], ROBOT_JOINT_MAX_RAD[i]); c->prev_abs_cur[i] = fabsf((float)in->current_ma[i]); }
    }
    c->torque_prev = true;
    c->initialised = true;
    out->torque_enable = true;

    /* ---- base target, sanitise, clamp, slew -------------------------------------- */
    bool limit_hit = false;
    for (int i = 0; i < N; i++) {
        float base;
        if (ROBOT_JOINT_IS_NECK[i] && !in->gaze_override && in->neck_valid) base = in->neck_target[i];
        else if (mode == ROBOT_CONTROL_MODE_ONNX_POLICY) base = in->pi_target[i];
        else base = ROBOT_JOINT_NEUTRAL_RAD[i];

        if (!isfinite(base)) { base = ROBOT_JOINT_NEUTRAL_RAD[i]; limit_hit = true; }
        float cl = clampf(base, ROBOT_JOINT_MIN_RAD[i], ROBOT_JOINT_MAX_RAD[i]);
        if (fabsf(cl - base) > 1e-4f) limit_hit = true;

        float step = ROBOT_JOINT_MAX_VEL_RAD_S[i] * dt;
        c->cmd[i] += clampf(cl - c->cmd[i], -step, step);
    }

    /* ---- reflex triggers --------------------------------------------------------- */
    if (in->looming_event) { c->flinch_active = true; c->flinch_t0 = in->now_us; }

    bool brace = false;
    for (int i = 0; i < N; i++) {
        float ac = fabsf((float)in->current_ma[i]);
        if (ROBOT_JOINT_IS_ANKLE[i] && ac - c->prev_abs_cur[i] > ROBOT_REFLEX_ANKLE_BRACE_CURRENT_SPIKE_MA) brace = true;
        c->prev_abs_cur[i] = ac;
    }
    if (in->imu_valid) {
        float gx = in->gyro_rad_s[0], gy = in->gyro_rad_s[1], gz = in->gyro_rad_s[2];
        if (sqrtf(gx * gx + gy * gy + gz * gz) > ROBOT_REFLEX_ANKLE_BRACE_GYRO_RAD_S) brace = true;
        /* cos(tilt) = z-component of world-up expressed in the body frame (3rd row of R) */
        float w = in->quat_wxyz[0], x = in->quat_wxyz[1], y = in->quat_wxyz[2], z = in->quat_wxyz[3];
        float cos_tilt = clampf(w * w - x * x - y * y + z * z, -1.0f, 1.0f);
        if (acosf(cos_tilt) * (180.0f / 3.14159265f) > ROBOT_REFLEX_ANKLE_BRACE_TILT_DEG) brace = true;
    }
    if (brace) { c->brace_active = true; c->brace_t0 = in->now_us; }

    float e_fl = envelope(c->flinch_active, in->now_us, c->flinch_t0, ROBOT_REFLEX_LOOMING_FLINCH_HOLD_MS, ROBOT_REFLEX_LOOMING_FLINCH_RELEASE_MS);
    float e_br = envelope(c->brace_active, in->now_us, c->brace_t0, ROBOT_REFLEX_ANKLE_BRACE_HOLD_MS, ROBOT_REFLEX_ANKLE_BRACE_RELEASE_MS);
    if (e_fl <= 0.0f) c->flinch_active = false; else mask |= ROBOT_REFLEX_BIT_LOOMING_FLINCH;
    if (e_br <= 0.0f) c->brace_active = false;  else mask |= ROBOT_REFLEX_BIT_ANKLE_BRACE;

    /* ---- compose, over-current clamp, hard limits (last) ------------------------- */
    for (int i = 0; i < N; i++) {
        float t = c->cmd[i] + e_fl * ROBOT_REFLEX_LOOMING_FLINCH_OFFSETS_RAD[i] + e_br * ROBOT_REFLEX_ANKLE_BRACE_OFFSETS_RAD[i];

        float ac = fabsf((float)in->current_ma[i]), lim = ROBOT_JOINT_MAX_CURRENT_MA[i];
        if (ac > lim) {
            c->oc_accum_us[i] += dt * 1e6f;
            if (c->oc_accum_us[i] >= ROBOT_SAFETY_OVERCURRENT_TRIP_MS * 1000.0f) c->oc_clamped[i] = true;
        } else if (ac < lim * ROBOT_SAFETY_OVERCURRENT_RELEASE) {
            c->oc_accum_us[i] = 0.0f;
            c->oc_clamped[i] = false;
        }
        if (c->oc_clamped[i]) {            /* stop pushing: target = where the joint is */
            t = in->pos[i];
            c->cmd[i] = in->pos[i];        /* and don't wind the command up behind it */
            mask |= ROBOT_REFLEX_BIT_OVERCURRENT_CLAMP;
        }

        float hard = clampf(t, ROBOT_JOINT_MIN_RAD[i], ROBOT_JOINT_MAX_RAD[i]);
        if (fabsf(hard - t) > 1e-4f) limit_hit = true;
        out->target[i] = hard;
    }
    if (limit_hit) mask |= ROBOT_REFLEX_BIT_LIMIT_CLAMP;
    out->reflex_mask = mask;
}
