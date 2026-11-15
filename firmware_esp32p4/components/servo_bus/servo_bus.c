#include "servo_bus.h"
#include "servo_drivers.h"
#include "servo_uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "servo_bus";
static servo_driver_t *s_drv;
static robot_joint_state_t s_last[ROBOT_NUM_JOINTS]; /* last good calibrated values */
static uint32_t s_fail[ROBOT_NUM_JOINTS];

esp_err_t servo_bus_init(void) {
    esp_err_t e = servo_uart_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(e)); return e; }
#if defined(ROBOT_BUS_DRIVER_FEETECH_STS)
    s_drv = servo_feetech_sts_driver();
#elif defined(ROBOT_BUS_DRIVER_DYNAMIXEL_X)
    s_drv = servo_dynamixel_x_driver();
#else
#error "robot_spec.yaml bus.driver has no matching servo driver"
#endif
    memset(s_last, 0, sizeof(s_last));
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) s_last[i].pos_rad = ROBOT_JOINT_NEUTRAL_RAD[i];
    ESP_LOGI(TAG, "driver=%s joints=%d baud=%d", s_drv->name, ROBOT_NUM_JOINTS, ROBOT_BUS_BAUD);
    return ESP_OK;
}

int servo_bus_read(robot_joint_state_t js[ROBOT_NUM_JOINTS], bool valid[ROBOT_NUM_JOINTS]) {
    servo_feedback_t fb[ROBOT_NUM_JOINTS];
    memset(fb, 0, sizeof(fb));
    (void)s_drv->read_all(s_drv, fb);
    int ok = 0;
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        if (fb[i].valid) {
            s_last[i].pos_rad = fb[i].pos_rad - ROBOT_JOINT_ZERO_OFFSET_RAD[i];
            s_last[i].vel_rad_s = fb[i].vel_rad_s;
            s_last[i].current_ma = fb[i].current_ma;
            s_fail[i] = 0;
            ok++;
        } else {
            s_fail[i]++;
        }
        valid[i] = fb[i].valid;
        js[i] = s_last[i];
    }
    return ok;
}

esp_err_t servo_bus_write(const float target_rad[ROBOT_NUM_JOINTS]) {
    float raw[ROBOT_NUM_JOINTS];
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) raw[i] = target_rad[i] + ROBOT_JOINT_ZERO_OFFSET_RAD[i];
    return s_drv->write_positions(s_drv, raw);
}

esp_err_t servo_bus_set_torque(bool enable) { return s_drv->set_torque(s_drv, enable); }

uint32_t servo_bus_fail_count(int joint) { return (joint >= 0 && joint < ROBOT_NUM_JOINTS) ? s_fail[joint] : 0; }
