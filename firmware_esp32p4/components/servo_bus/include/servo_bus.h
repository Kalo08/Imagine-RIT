/*
 * servo_bus -- modular driver layer for daisy-chained half-duplex TTL serial servos.
 *
 * The rest of the firmware only sees servo_bus_*(): joint-index based, radians, with the
 * per-joint zero offsets from calibration applied. Protocol specifics live behind
 * servo_driver_t (Feetech STS/SCS "servo_feetech.c", Dynamixel Protocol 2.0 "servo_dynamixel.c").
 * Adding another vendor = implement servo_driver_t and add a case in servo_bus_init().
 *
 * Threading: NOT thread-safe by design. The Core-1 control task is the only caller.
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "robot_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pos_rad;      /* raw servo angle, centre tick == 0 rad (offset NOT applied at driver level) */
    float vel_rad_s;
    int16_t current_ma; /* signed: sign follows load direction where the servo reports it */
    bool valid;         /* servo answered this cycle */
} servo_feedback_t;

typedef struct servo_driver servo_driver_t;
struct servo_driver {
    const char *name;
    /* Read every servo (by ROBOT_JOINT_ID[i]); fb[i].valid=false on timeout/CRC error. */
    esp_err_t (*read_all)(servo_driver_t *d, servo_feedback_t fb[ROBOT_NUM_JOINTS]);
    /* Write all goal positions in one broadcast frame (raw rad, centre == 0). */
    esp_err_t (*write_positions)(servo_driver_t *d, const float raw_rad[ROBOT_NUM_JOINTS]);
    /* Torque on/off for all servos. */
    esp_err_t (*set_torque)(servo_driver_t *d, bool enable);
};

/* Selects the driver from ROBOT_BUS_DRIVER_* (robot_spec.yaml) and configures the UART (Kconfig). */
esp_err_t servo_bus_init(void);

/* Feedback with calibration applied: pos = raw - ZERO_OFFSET. Joints that failed keep their
 * last good values and get valid=false. Returns the number of joints that answered. */
int servo_bus_read(robot_joint_state_t js[ROBOT_NUM_JOINTS], bool valid[ROBOT_NUM_JOINTS]);

/* Targets in calibrated radians (offset re-applied internally). */
esp_err_t servo_bus_write(const float target_rad[ROBOT_NUM_JOINTS]);

esp_err_t servo_bus_set_torque(bool enable);

/* Consecutive-failure counters, for diagnostics. */
uint32_t servo_bus_fail_count(int joint);

#ifdef __cplusplus
}
#endif
