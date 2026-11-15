/*
 * DYNAMIXEL Protocol 2.0 driver (X-series: XL330, XL430, XM430 ... Position Control mode).
 *
 * Instruction packet: FF FF FD 00 ID LEN_L LEN_H INST PARAMS... CRC_L CRC_H
 *   LEN = 1 (INST) + nparams + 2 (CRC);  CRC-16 poly 0x8005 (BUYPASS), init 0, over everything before it.
 *   Byte stuffing: any FF FF FD in the PARAMS is followed by an extra FD (counted in LEN, covered by CRC).
 * Control table (X-series): 64 torque enable (1B), 116 goal position (4B),
 *   126 present current (2B, 2.69 mA/LSB on XL430/XM430; check your model), 128 velocity (4B, 0.229 rpm/LSB),
 *   132 position (4B, 4096 ticks/rev, 2048 centre).
 * Uses Sync Read (0x82) so all joints answer within one bus round trip.
 * The servos must already be in Position Control mode (operating mode 3) -- configure once with Wizard/R+.
 */
#include "servo_drivers.h"
#include "servo_uart.h"
#include <math.h>
#include <string.h>

#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83
#define INST_WRITE 0x03
#define REG_TORQUE 64
#define REG_GOAL_POS 116
#define REG_PRESENT_CURRENT 126
#define PRESENT_LEN 10 /* current(2) + velocity(4) + position(4) */
#define CURRENT_LSB_MA 2.69f
#define VEL_LSB_RAD_S (0.229f * 6.2831853f / 60.0f)
#define TWO_PI 6.2831853f
#define RX_TIMEOUT_US 2500

static uint16_t crc16_dxl(uint16_t crc, const uint8_t *p, size_t n) {
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Build + send an instruction packet, applying byte stuffing to params. */
static esp_err_t send_inst(uint8_t id, uint8_t inst, const uint8_t *params, size_t np) {
    uint8_t buf[16 + 2 * (4 + 9 * ROBOT_NUM_JOINTS)];
    size_t n = 0;
    buf[n++] = 0xFF; buf[n++] = 0xFF; buf[n++] = 0xFD; buf[n++] = 0x00;
    buf[n++] = id;
    size_t len_pos = n; n += 2;
    buf[n++] = inst;
    size_t start = n;
    for (size_t i = 0; i < np; i++) {
        if (n + 4 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
        buf[n++] = params[i];
        if (i >= 2 && params[i] == 0xFD && params[i - 1] == 0xFF && params[i - 2] == 0xFF) buf[n++] = 0xFD;
    }
    uint16_t len = (uint16_t)((n - start) + 1 + 2);
    buf[len_pos] = (uint8_t)(len & 0xFF);
    buf[len_pos + 1] = (uint8_t)(len >> 8);
    uint16_t crc = crc16_dxl(0, buf, n);
    buf[n++] = (uint8_t)(crc & 0xFF);
    buf[n++] = (uint8_t)(crc >> 8);
    return servo_uart_write(buf, n);
}

/* Read one status packet; unstuffs params into out (expects `expect` bytes). */
static bool read_status(uint8_t id, uint8_t *out, size_t expect) {
    uint8_t h[9]; /* FF FF FD 00 ID LEN_L LEN_H INST ERR */
    if (servo_uart_read(h, 7, RX_TIMEOUT_US) != 7) return false;
    if (h[0] != 0xFF || h[1] != 0xFF || h[2] != 0xFD || h[3] != 0x00 || h[4] != id) return false;
    uint16_t len = (uint16_t)(h[5] | (h[6] << 8));
    if (len < 4 || len > 2 + 2 * (PRESENT_LEN + 2) + 4) return false;
    uint8_t body[2 * (PRESENT_LEN + 2) + 8];
    if (len > sizeof(body) || servo_uart_read(body, len, RX_TIMEOUT_US) != len) return false;
    uint16_t crc = crc16_dxl(crc16_dxl(0, h, 7), body, (size_t)len - 2);
    if (crc != (uint16_t)(body[len - 2] | (body[len - 1] << 8))) return false;
    if (body[0] != 0x55 || body[1] != 0) return false; /* STATUS inst, no hardware alert/error */
    size_t got = 0;
    for (size_t i = 2; i < (size_t)len - 2 && got < expect; i++) {
        if (i >= 4 && body[i] == 0xFD && body[i - 1] == 0xFF && body[i - 2] == 0xFF) continue; /* stuffed byte */
        out[got++] = body[i];
    }
    return got == expect;
}

static esp_err_t dxl_read_all(servo_driver_t *d, servo_feedback_t fb[ROBOT_NUM_JOINTS]) {
    (void)d;
    uint8_t p[4 + ROBOT_NUM_JOINTS] = {REG_PRESENT_CURRENT & 0xFF, REG_PRESENT_CURRENT >> 8, PRESENT_LEN, 0};
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) p[4 + i] = ROBOT_JOINT_ID[i];
    servo_uart_flush_rx();
    esp_err_t e = send_inst(0xFE, INST_SYNC_READ, p, 4 + ROBOT_NUM_JOINTS);
    if (e != ESP_OK) return e;
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        uint8_t b[PRESENT_LEN];
        fb[i].valid = read_status(ROBOT_JOINT_ID[i], b, PRESENT_LEN);
        if (!fb[i].valid) continue;
        int16_t cur = (int16_t)(b[0] | (b[1] << 8));
        int32_t vel = (int32_t)((uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24));
        int32_t pos = (int32_t)((uint32_t)b[6] | ((uint32_t)b[7] << 8) | ((uint32_t)b[8] << 16) | ((uint32_t)b[9] << 24));
        fb[i].current_ma = (int16_t)lroundf(fminf(32767.0f, fmaxf(-32768.0f, cur * CURRENT_LSB_MA)));
        fb[i].vel_rad_s = vel * VEL_LSB_RAD_S;
        fb[i].pos_rad = (float)(pos - 2048) * (TWO_PI / 4096.0f);
    }
    return ESP_OK;
}

static esp_err_t dxl_write_positions(servo_driver_t *d, const float raw_rad[ROBOT_NUM_JOINTS]) {
    (void)d;
    uint8_t p[4 + 5 * ROBOT_NUM_JOINTS];
    size_t n = 0;
    p[n++] = REG_GOAL_POS & 0xFF; p[n++] = REG_GOAL_POS >> 8; p[n++] = 4; p[n++] = 0;
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        int32_t t = 2048 + (int32_t)lroundf(raw_rad[i] * (4096.0f / TWO_PI));
        if (t < 0) t = 0;
        if (t > 4095) t = 4095;
        p[n++] = ROBOT_JOINT_ID[i];
        p[n++] = (uint8_t)(t & 0xFF); p[n++] = (uint8_t)((t >> 8) & 0xFF); p[n++] = 0; p[n++] = 0;
    }
    return send_inst(0xFE, INST_SYNC_WRITE, p, n);
}

static esp_err_t dxl_set_torque(servo_driver_t *d, bool enable) {
    (void)d;
    uint8_t p[4 + 2 * ROBOT_NUM_JOINTS];
    size_t n = 0;
    p[n++] = REG_TORQUE; p[n++] = 0; p[n++] = 1; p[n++] = 0;
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) { p[n++] = ROBOT_JOINT_ID[i]; p[n++] = enable ? 1 : 0; }
    return send_inst(0xFE, INST_SYNC_WRITE, p, n);
}

static servo_driver_t s_driver = {
    .name = "dynamixel_x", .read_all = dxl_read_all,
    .write_positions = dxl_write_positions, .set_torque = dxl_set_torque,
};
servo_driver_t *servo_dynamixel_x_driver(void) { return &s_driver; }
