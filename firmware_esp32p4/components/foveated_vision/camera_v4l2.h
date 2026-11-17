/* Private: MIPI-CSI camera via esp_video's V4L2 interface (RGB565, MMAP buffers). */
#pragma once
#include <stdint.h>
#include "esp_err.h"

#define CAM_W 640
#define CAM_H 480

esp_err_t camera_open(void);
/* Blocks for the next frame. Returns a pointer to RGB565 pixels valid until camera_release(). */
const uint16_t *camera_grab(void);
void camera_release(void);
