/*
 * imu_bno085 -- Bosch/CEVA BNO085 over I2C using the SHTP transport.
 * Enables Game Rotation Vector (no magnetometer: immune to indoor field distortion, ideal for
 * balance) and calibrated Gyroscope at 4 ms. Poll from the Core-1 loop; each call drains
 * whatever the sensor has queued.
 *
 * NOTE: I2C @400 kHz costs ~1 ms per report pair. If the 200 Hz budget gets tight, switch to
 * the BNO085's SPI interface (same SHTP framing) -- only imu_bno085.c's xfer functions change.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float quat_wxyz[4];
    float gyro_rad_s[3];
    int64_t quat_time_us;  /* esp_timer time of last quaternion update (0 = never) */
    int64_t gyro_time_us;
} imu_sample_t;

esp_err_t imu_bno085_init(void);
/* Drain pending sensor packets (non-blocking beyond I2C transfer time). Returns packets consumed. */
int imu_bno085_poll(void);
/* Copy out the latest sample. Returns true if both quat and gyro were updated within max_age_us. */
bool imu_bno085_get(imu_sample_t *out, int64_t max_age_us);

#ifdef __cplusplus
}
#endif
