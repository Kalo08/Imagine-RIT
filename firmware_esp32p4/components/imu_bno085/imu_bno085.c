#include "imu_bno085.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "bno085";

/* SHTP channels / report ids (BNO080/085 datasheet + SH-2 reference manual) */
#define CH_COMMAND 0
#define CH_CONTROL 2
#define CH_SENSOR 3
#define REPORT_SET_FEATURE 0xFD
#define REPORT_BASE_TIMESTAMP 0xFB
#define REPORT_TIMESTAMP_REBASE 0xFA
#define REPORT_GYRO 0x02
#define REPORT_GAME_ROT 0x08
#define REPORT_INTERVAL_US 4000

#define Q14 (1.0f / 16384.0f)
#define Q9 (1.0f / 512.0f)

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_seq[8];
static imu_sample_t s_s = {.quat_wxyz = {1, 0, 0, 0}};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_rx[272];

static esp_err_t shtp_send(uint8_t channel, const uint8_t *payload, uint16_t len) {
    uint8_t buf[4 + 24];
    if (len + 4u > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    uint16_t total = len + 4;
    buf[0] = total & 0xFF;
    buf[1] = total >> 8;
    buf[2] = channel;
    buf[3] = s_seq[channel]++;
    memcpy(&buf[4], payload, len);
    return i2c_master_transmit(s_dev, buf, total, 20);
}

static esp_err_t set_feature(uint8_t report_id, uint32_t interval_us) {
    uint8_t p[17] = {REPORT_SET_FEATURE, report_id, 0, 0, 0};
    memcpy(&p[5], &interval_us, 4); /* little-endian host */
    return shtp_send(CH_CONTROL, p, sizeof(p));
}

static inline int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* Parse one channel-3 payload: [timebase(5)] then packed reports. */
static void parse_sensor_reports(const uint8_t *p, int len) {
    int64_t now = esp_timer_get_time();
    int i = 0;
    while (i < len) {
        switch (p[i]) {
        case REPORT_BASE_TIMESTAMP:
        case REPORT_TIMESTAMP_REBASE:
            i += 5;
            break;
        case REPORT_GAME_ROT:
            if (i + 12 > len) return;
            portENTER_CRITICAL(&s_lock);
            s_s.quat_wxyz[1] = le16(&p[i + 4]) * Q14;  /* i */
            s_s.quat_wxyz[2] = le16(&p[i + 6]) * Q14;  /* j */
            s_s.quat_wxyz[3] = le16(&p[i + 8]) * Q14;  /* k */
            s_s.quat_wxyz[0] = le16(&p[i + 10]) * Q14; /* real */
            s_s.quat_time_us = now;
            portEXIT_CRITICAL(&s_lock);
            i += 12;
            break;
        case REPORT_GYRO:
            if (i + 10 > len) return;
            portENTER_CRITICAL(&s_lock);
            s_s.gyro_rad_s[0] = le16(&p[i + 4]) * Q9;
            s_s.gyro_rad_s[1] = le16(&p[i + 6]) * Q9;
            s_s.gyro_rad_s[2] = le16(&p[i + 8]) * Q9;
            s_s.gyro_time_us = now;
            portEXIT_CRITICAL(&s_lock);
            i += 10;
            break;
        default:
            return; /* unknown report: sizes unknown, stop parsing this packet */
        }
    }
}

int imu_bno085_poll(void) {
    int consumed = 0;
    /* INT is active-low while the sensor has data queued. */
    for (int n = 0; n < 4 && gpio_get_level(CONFIG_IMU_BNO085_INT_GPIO) == 0; n++) {
        uint8_t hdr[4];
        if (i2c_master_receive(s_dev, hdr, 4, 5) != ESP_OK) break;
        uint16_t len = (uint16_t)(hdr[0] | ((hdr[1] & 0x7F) << 8));
        if (len <= 4) break;
        if (len > sizeof(s_rx)) len = sizeof(s_rx); /* oversize (e.g. advertisement): read & discard what fits */
        if (i2c_master_receive(s_dev, s_rx, len, 10) != ESP_OK) break;
        consumed++;
        if (s_rx[2] == CH_SENSOR) parse_sensor_reports(&s_rx[4], (int)len - 4);
    }
    return consumed;
}

bool imu_bno085_get(imu_sample_t *out, int64_t max_age_us) {
    portENTER_CRITICAL(&s_lock);
    *out = s_s;
    portEXIT_CRITICAL(&s_lock);
    int64_t now = esp_timer_get_time();
    return out->quat_time_us && out->gyro_time_us && now - out->quat_time_us <= max_age_us &&
           now - out->gyro_time_us <= max_age_us;
}

esp_err_t imu_bno085_init(void) {
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_IMU_BNO085_I2C_PORT,
        .sda_io_num = CONFIG_IMU_BNO085_SDA_GPIO,
        .scl_io_num = CONFIG_IMU_BNO085_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t e = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (e != ESP_OK) return e;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_IMU_BNO085_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if ((e = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev)) != ESP_OK) return e;

    gpio_config_t in = {.pin_bit_mask = 1ULL << CONFIG_IMU_BNO085_INT_GPIO, .mode = GPIO_MODE_INPUT, .pull_up_en = 1};
    gpio_config_t out = {.pin_bit_mask = 1ULL << CONFIG_IMU_BNO085_RST_GPIO, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&in);
    gpio_config(&out);

    /* Hardware reset, then wait for the sensor to boot and flush its advertisement/reset reports. */
    gpio_set_level(CONFIG_IMU_BNO085_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_IMU_BNO085_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int i = 0; i < 16; i++) imu_bno085_poll();

    if ((e = set_feature(REPORT_GAME_ROT, REPORT_INTERVAL_US)) != ESP_OK) { ESP_LOGE(TAG, "enable game rot failed"); return e; }
    if ((e = set_feature(REPORT_GYRO, REPORT_INTERVAL_US)) != ESP_OK) { ESP_LOGE(TAG, "enable gyro failed"); return e; }
    ESP_LOGI(TAG, "BNO085 ready (addr 0x%02x, %d us reports)", CONFIG_IMU_BNO085_I2C_ADDR, REPORT_INTERVAL_US);
    return ESP_OK;
}
