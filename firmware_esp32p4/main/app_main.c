/*
 * ESP32-P4 "Spinal Cord & Visual Cortex" -- task layout
 *
 *   Core 1 (hard real-time)   control_task  prio MAX-2, 200 Hz via vTaskDelayUntil
 *                              IMU poll -> servo bus read -> spinal_reflex_step -> servo bus write
 *                              -> StatePacket to usb_bridge.  The ONLY task that touches the servo UART.
 *   Core 0 (vision/UI/USB)    vision_task   (foveated_vision)   camera -> tracker -> neck PID mailbox
 *                              usb_tx task   (usb_bridge)        queue -> CDC; TinyUSB device task
 *                              lvgl task     (esp_lvgl_port)     60 FPS face_ui
 *                              ui_feed_task                      snapshot -> face_ui_update
 *
 * Subsystems that fail to start (no camera, no display, no IMU) are reported and skipped; the
 * control loop keeps running with the affected feature disabled (IMU stale => IMU reflexes off,
 * no vision => neck follows Pi / neutral). Servo bus or USB failure is fatal (restart).
 */
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#include "face_ui.h"
#include "foveated_vision.h"
#include "imu_bno085.h"
#include "robot_protocol.h"
#include "servo_bus.h"
#include "spinal_reflex.h"
#include "usb_bridge.h"

lv_display_t *board_display_init(void); /* board_display.c */

static const char *TAG = "app";
_Static_assert(1000 % ROBOT_STATE_HZ == 0, "state_hz must divide 1000 (FreeRTOS tick is 1 kHz)");

static volatile uint32_t s_overruns;

static void control_task(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    static spinal_ctx_t ctx;
    spinal_reflex_init(&ctx);
    bool torque_state = false;
    const TickType_t period = pdMS_TO_TICKS(1000 / ROBOT_STATE_HZ);
    TickType_t last = xTaskGetTickCount();
    uint32_t ack = 0;

    for (;;) {
        vTaskDelayUntil(&last, period);
        const int64_t t0 = esp_timer_get_time();

        /* ---- sense ---- */
        imu_bno085_poll();
        imu_sample_t imu;
        const bool imu_ok = imu_bno085_get(&imu, (int64_t)ROBOT_SAFETY_IMU_STALE_MS * 1000);
        robot_joint_state_t js[ROBOT_NUM_JOINTS];
        bool valid[ROBOT_NUM_JOINTS];
        const int answered = servo_bus_read(js, valid);

        robot_action_packet_t act;
        uint32_t age_ms = 0;
        const bool have = usb_bridge_get_action(&act, &age_ms);
        const bool alive = have && age_ms <= ROBOT_ACTION_TIMEOUT_MS;
        if (have) ack = act.sequence_id;

        /* ---- decide ---- */
        static spinal_in_t in; /* static: ~0.5 KB, keeps the task stack small */
        static spinal_out_t out;
        in.now_us = (uint64_t)t0;
        in.pi_mode = have ? (robot_control_mode_t)act.control_mode : ROBOT_CONTROL_MODE_NEUTRAL_HOLD;
        in.pi_link_alive = alive;
        in.gaze_override = have && act.gaze_override;
        for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
            in.pi_target[i] = have ? act.target_pos_rad[i] : ROBOT_JOINT_NEUTRAL_RAD[i];
            in.pos[i] = js[i].pos_rad;
            in.vel[i] = js[i].vel_rad_s;
            in.current_ma[i] = js[i].current_ma;
        }
        in.servo_fault = answered < ROBOT_NUM_JOINTS;
        in.imu_valid = imu_ok;
        if (imu_ok) { memcpy(in.quat_wxyz, imu.quat_wxyz, sizeof(in.quat_wxyz)); memcpy(in.gyro_rad_s, imu.gyro_rad_s, sizeof(in.gyro_rad_s)); }
        in.neck_valid = foveated_vision_get_neck_targets(in.neck_target);
        in.looming_event = foveated_vision_take_looming();
        {
            float p[ROBOT_NUM_JOINTS], v[ROBOT_NUM_JOINTS];
            for (int i = 0; i < ROBOT_NUM_JOINTS; i++) { p[i] = js[i].pos_rad; v[i] = js[i].vel_rad_s; }
            foveated_vision_update_neck_feedback(p, v);
        }
        spinal_reflex_step(&ctx, &in, &out);

        /* ---- act ---- */
        if (out.torque_enable != torque_state) {
            if (servo_bus_set_torque(out.torque_enable) == ESP_OK) torque_state = out.torque_enable;
        }
        if (torque_state) (void)servo_bus_write(out.target);

        /* ---- report ---- */
        static robot_state_packet_t st;
        st.timestamp_us = (uint64_t)t0;
        st.ack_sequence_id = ack;
        st.active_mode = (uint8_t)out.active_mode;
        if (imu_ok) { memcpy(st.quat_wxyz, imu.quat_wxyz, sizeof(st.quat_wxyz)); memcpy(st.gyro_rad_s, imu.gyro_rad_s, sizeof(st.gyro_rad_s)); }
        else { st.quat_wxyz[0] = 1; st.quat_wxyz[1] = st.quat_wxyz[2] = st.quat_wxyz[3] = 0; memset(st.gyro_rad_s, 0, sizeof(st.gyro_rad_s)); }
        memcpy(st.joints, js, sizeof(st.joints));
        st.reflex_mask = out.reflex_mask;
        fovea_gaze_t g;
        foveated_vision_get_gaze(&g);
        st.gaze_x = g.x; st.gaze_y = g.y; st.gaze_confidence = g.confidence; st.gaze_locked = g.locked ? 1 : 0;
        usb_bridge_push_state(&st);

        if (esp_timer_get_time() - t0 > (int64_t)period * 1000) s_overruns++;
        esp_task_wdt_reset();
    }
}

/* 30 Hz snapshot of state + gaze for the face UI (kept off Core 1). */
static void ui_feed_task(void *arg) {
    (void)arg;
    for (;;) {
        robot_state_packet_t s;
        if (usb_bridge_peek_state(&s)) {
            face_ui_inputs_t in = {.gaze_x = s.gaze_x, .gaze_y = s.gaze_y, .gaze_conf = s.gaze_confidence,
                                   .gaze_locked = s.gaze_locked, .reflex_mask = s.reflex_mask};
            for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
                if (!ROBOT_JOINT_IS_ANKLE[i]) continue;
                float l = fabsf((float)s.joints[i].current_ma) / ROBOT_JOINT_MAX_CURRENT_MA[i];
                if (l > in.foot_load) in.foot_load = l;
            }
            face_ui_update(&in);
        }
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "robot '%s': %d joints, spec hash %s", "desktop_biped", ROBOT_NUM_JOINTS, ROBOT_SPEC_HASH);

    ESP_ERROR_CHECK(servo_bus_init());                    /* fatal: nothing to control without the bus */
    if (imu_bno085_init() != ESP_OK) ESP_LOGE(TAG, "IMU init failed: IMU-driven reflexes disabled");
    ESP_ERROR_CHECK(usb_bridge_start(0));                 /* fatal: no link to the Pi */

    foveated_vision_set_roi_sink(usb_bridge_push_roi);
    if (foveated_vision_start(0, NULL) != ESP_OK) ESP_LOGE(TAG, "vision disabled");

    lv_display_t *disp = board_display_init();
    if (disp && face_ui_start(disp) == ESP_OK) xTaskCreatePinnedToCore(ui_feed_task, "ui_feed", 3072, NULL, 4, NULL, 0);
    else ESP_LOGE(TAG, "face UI disabled");

    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}
