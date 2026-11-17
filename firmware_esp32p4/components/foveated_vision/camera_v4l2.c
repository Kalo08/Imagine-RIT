/*
 * Camera bring-up is the most board/sensor/esp_video-version-specific code in this repo.
 * Everything that depends on it is confined to this file. Written against the esp_video
 * component's documented capture flow (esp_video_init -> /dev/video0 -> S_FMT -> REQBUFS ->
 * QBUF -> STREAMON -> DQBUF); if your esp_video release renames config fields, adapt open only.
 */
#include "camera_v4l2.h"
#include "esp_log.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"
#include "sdkconfig.h"
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

static const char *TAG = "camera";
#define NBUF 2
static int s_fd = -1;
static void *s_buf[NBUF];
static size_t s_len[NBUF];
static int s_cur = -1;

esp_err_t camera_open(void) {
    const esp_video_init_csi_config_t csi = {
        .sccb_config = {.init_sccb = true, .i2c_config = {.port = CONFIG_CAMERA_SCCB_I2C_PORT, .scl_pin = CONFIG_CAMERA_SCCB_SCL_GPIO, .sda_pin = CONFIG_CAMERA_SCCB_SDA_GPIO}, .freq = 100000},
        .reset_pin = -1, .pwdn_pin = -1,
    };
    const esp_video_init_config_t cfg = {.csi = &csi};
    esp_err_t e = esp_video_init(&cfg);
    if (e != ESP_OK) { ESP_LOGE(TAG, "esp_video_init: %s", esp_err_to_name(e)); return e; }

    s_fd = open("/dev/video0", O_RDWR);
    if (s_fd < 0) { ESP_LOGE(TAG, "open /dev/video0 failed"); return ESP_FAIL; }

    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    fmt.fmt.pix.width = CAM_W;
    fmt.fmt.pix.height = CAM_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) { ESP_LOGE(TAG, "S_FMT failed"); return ESP_FAIL; }

    struct v4l2_requestbuffers req = {.count = NBUF, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    if (ioctl(s_fd, VIDIOC_REQBUFS, &req) != 0) { ESP_LOGE(TAG, "REQBUFS failed"); return ESP_FAIL; }
    for (int i = 0; i < NBUF; i++) {
        struct v4l2_buffer b = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i};
        if (ioctl(s_fd, VIDIOC_QUERYBUF, &b) != 0) return ESP_FAIL;
        s_len[i] = b.length;
        s_buf[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_fd, b.m.offset);
        if (s_buf[i] == MAP_FAILED) return ESP_FAIL;
        if (ioctl(s_fd, VIDIOC_QBUF, &b) != 0) return ESP_FAIL;
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_fd, VIDIOC_STREAMON, &type) != 0) { ESP_LOGE(TAG, "STREAMON failed"); return ESP_FAIL; }
    ESP_LOGI(TAG, "streaming %dx%d RGB565", CAM_W, CAM_H);
    return ESP_OK;
}

const uint16_t *camera_grab(void) {
    struct v4l2_buffer b = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    if (ioctl(s_fd, VIDIOC_DQBUF, &b) != 0) return NULL;
    s_cur = (int)b.index;
    return (const uint16_t *)s_buf[s_cur];
}

void camera_release(void) {
    if (s_cur < 0) return;
    struct v4l2_buffer b = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)s_cur};
    ioctl(s_fd, VIDIOC_QBUF, &b);
    s_cur = -1;
}
