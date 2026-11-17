#include "usb_bridge.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "usb_bridge";
#define ITF TINYUSB_CDC_ACM_0

static robot_parser_t s_parser;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static robot_action_packet_t s_action;
static bool s_action_valid;
static int64_t s_action_rx_us;
static robot_state_packet_t s_latest_state;
static bool s_state_valid;
static QueueHandle_t s_state_q;
static TaskHandle_t s_tx_task;
static robot_roi_frame_t s_roi;
static volatile bool s_roi_pending;
static usb_bridge_stats_t s_stats;

/* ---------------------------------------------------------------- RX (TinyUSB task context) */
static void on_rx(int itf, cdcacm_event_t *event) {
    (void)event;
    uint8_t buf[256];
    size_t n = 0;
    while (tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, buf, sizeof(buf), &n) == ESP_OK && n > 0) {
        for (size_t i = 0; i < n; i++) {
            if (!robot_parser_feed(&s_parser, buf[i])) continue;
            if (s_parser.type != ROBOT_MSG_ACTION_PACKET) continue; /* length already verified by the parser */
            robot_action_packet_t a;
            memcpy(&a, s_parser.payload, sizeof(a));
            portENTER_CRITICAL(&s_lock);
            s_action = a;
            s_action_valid = true;
            s_action_rx_us = esp_timer_get_time();
            portEXIT_CRITICAL(&s_lock);
        }
        s_stats.rx_frames = s_parser.frames_ok;
        s_stats.rx_crc_errors = s_parser.crc_errors;
        s_stats.rx_len_errors = s_parser.len_errors;
    }
}

/* ---------------------------------------------------------------- TX task (Core 0) */
static void send_frame(uint8_t type, const void *payload, uint16_t len) {
    uint8_t frame[ROBOT_MAX_FRAME];
    size_t n = robot_frame_encode(type, payload, len, frame, sizeof(frame));
    if (!n) return;
    /* If the CDC TX buffer is full only part of the frame is queued; the Pi's parser detects the
     * truncated frame by CRC/length and resyncs on the next magic. The loss is counted here. */
    if (tinyusb_cdcacm_write_queue(ITF, frame, n) != n) {
        s_stats.tx_dropped_usb++;
        return;
    }
    tinyusb_cdcacm_write_flush(ITF, 0);
    s_stats.tx_frames++;
}

static void tx_task(void *arg) {
    (void)arg;
    robot_state_packet_t s;
    for (;;) {
        if (xQueueReceive(s_state_q, &s, pdMS_TO_TICKS(20)) == pdTRUE) send_frame(ROBOT_MSG_STATE_PACKET, &s, sizeof(s));
        if (s_roi_pending) {
            send_frame(ROBOT_MSG_ROI_FRAME, &s_roi, sizeof(s_roi));
            s_roi_pending = false;
        }
    }
}

/* ---------------------------------------------------------------- public API */
esp_err_t usb_bridge_start(int tx_task_core) {
    robot_parser_init(&s_parser);
    s_state_q = xQueueCreate(8, sizeof(robot_state_packet_t));
    if (!s_state_q) return ESP_ERR_NO_MEM;

    const tinyusb_config_t tusb_cfg = {0}; /* default descriptors: CDC-ACM on the HS port (sdkconfig) */
    esp_err_t e = tinyusb_driver_install(&tusb_cfg);
    if (e != ESP_OK) return e;
    const tinyusb_config_cdcacm_t acm = {
        .usb_dev = TINYUSB_USBDEV_0, .cdc_port = ITF, .rx_unread_buf_sz = 512,
        .callback_rx = on_rx, .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL, .callback_line_coding_changed = NULL,
    };
    if ((e = tusb_cdc_acm_init(&acm)) != ESP_OK) return e;
    xTaskCreatePinnedToCore(tx_task, "usb_tx", 4096, NULL, 10, &s_tx_task, tx_task_core);
    ESP_LOGI(TAG, "USB CDC-ACM up; frame sizes: state=%d action=%d roi=%d", (int)sizeof(robot_state_packet_t),
             (int)sizeof(robot_action_packet_t), (int)sizeof(robot_roi_frame_t));
    return ESP_OK;
}

void usb_bridge_push_state(const robot_state_packet_t *s) {
    portENTER_CRITICAL(&s_lock);
    s_latest_state = *s;
    s_state_valid = true;
    portEXIT_CRITICAL(&s_lock);
    if (xQueueSend(s_state_q, s, 0) != pdTRUE) { /* full: drop OLDEST so the Pi always gets fresh data */
        robot_state_packet_t old;
        (void)xQueueReceive(s_state_q, &old, 0);
        (void)xQueueSend(s_state_q, s, 0);
        s_stats.tx_dropped_state++;
    }
}

void usb_bridge_push_roi(const robot_roi_frame_t *r) {
    if (s_roi_pending) return;
    s_roi = *r;
    s_roi_pending = true;
}

bool usb_bridge_get_action(robot_action_packet_t *out, uint32_t *age_ms) {
    bool ok;
    int64_t t;
    portENTER_CRITICAL(&s_lock);
    ok = s_action_valid;
    if (ok) *out = s_action;
    t = s_action_rx_us;
    portEXIT_CRITICAL(&s_lock);
    if (ok) {
        int64_t age = (esp_timer_get_time() - t) / 1000;
        *age_ms = age < 0 ? 0 : (age > UINT32_MAX ? UINT32_MAX : (uint32_t)age);
    }
    return ok;
}

bool usb_bridge_peek_state(robot_state_packet_t *out) {
    bool ok;
    portENTER_CRITICAL(&s_lock);
    ok = s_state_valid;
    if (ok) *out = s_latest_state;
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

void usb_bridge_get_stats(usb_bridge_stats_t *s) { *s = s_stats; }
