/*
 * spinal_reflex -- hard safety + sub-10 ms reflexes. Pure C, no ESP-IDF dependencies,
 * so it can be unit-tested on a host compiler. Runs once per Core-1 tick (200 Hz = 5 ms).
 *
 * Pipeline per tick (order matters; the hard limit clamp is ALWAYS last):
 *   mode arbitration (ESTOP latch, Pi watchdog)
 *   -> base target (neutral | Pi target | vision neck target)
 *   -> sanitise (NaN/Inf) + clamp -> slew limit
 *   -> + reflex offsets (looming flinch, ankle brace)         [bypass the slew limit]
 *   -> over-current clamp
 *   -> hard min/max clamp
 */
#pragma once
#include "robot_protocol.h"

typedef struct {
    uint64_t now_us;

    robot_control_mode_t pi_mode;   /* mode from the last ActionPacket */
    bool pi_link_alive;             /* a valid ActionPacket arrived within ROBOT_ACTION_TIMEOUT_MS */
    bool gaze_override;             /* Pi drives neck joints itself */
    float pi_target[ROBOT_NUM_JOINTS];

    bool neck_valid;                /* vision core produced neck targets */
    float neck_target[ROBOT_NUM_JOINTS]; /* only entries with ROBOT_JOINT_IS_NECK are used */

    float pos[ROBOT_NUM_JOINTS];
    float vel[ROBOT_NUM_JOINTS];
    int16_t current_ma[ROBOT_NUM_JOINTS];
    bool servo_fault;               /* at least one servo did not answer this tick */

    bool imu_valid;
    float quat_wxyz[4];
    float gyro_rad_s[3];

    bool looming_event;             /* one-shot from the vision core */
} spinal_in_t;

typedef struct {
    float target[ROBOT_NUM_JOINTS]; /* final joint targets (rad), already hard-clamped */
    bool torque_enable;
    robot_control_mode_t active_mode;
    uint16_t reflex_mask;           /* ROBOT_REFLEX_BIT_* flags */
} spinal_out_t;

typedef struct {
    bool initialised, torque_prev, estop_latched;
    uint64_t last_us;
    float cmd[ROBOT_NUM_JOINTS];          /* slew-limited base command */
    float prev_abs_cur[ROBOT_NUM_JOINTS];
    float oc_accum_us[ROBOT_NUM_JOINTS];
    bool oc_clamped[ROBOT_NUM_JOINTS];
    bool flinch_active, brace_active;
    uint64_t flinch_t0, brace_t0;
} spinal_ctx_t;

void spinal_reflex_init(spinal_ctx_t *c);
void spinal_reflex_step(spinal_ctx_t *c, const spinal_in_t *in, spinal_out_t *out);
