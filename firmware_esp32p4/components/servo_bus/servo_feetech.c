/*
 * Feetech STS3215-class (little-endian SCS/STS protocol) driver.
 *
 * Packet:  FF FF ID LEN INSTR PARAMS... CHK     LEN = nparams + 2, CHK = ~(ID+LEN+INSTR+sum(PARAMS))
 * Reply:   FF FF ID LEN ERR DATA... CHK         LEN = ndata + 2
 *
 * STS3215 control table used (verify against your servo's datasheet/firmware revision):
 *   40 torque enable (1B)      42 goal position (2B, +time 44, +speed 46)
 *   56 present position (2B)   58 present speed (2B, bit15 = sign)
 *   60 present load (2B)       69 present current (2B, 6.5 mA/LSB)
 * 4096 ticks per revolution; 2048 is the mechanical centre.
 *
 * Reads are one READ transaction per servo (works on every firmware revision). At 1 Mbaud a
 * 15-byte read costs ~0.35 ms, so ~10 servos fit comfortably inside the 5 ms control period;
 * ROBOT_NUM_JOINTS above ~14 would need sync-read (not all STS firmwares support it).
 */
#include "servo_drivers.h"
#include "servo_uart.h"
#include <math.h>
#include <string.h>

#define TICKS_PER_REV 4096.0f
#define CENTRE_TICK 2048
#define TWO_PI 6.2831853f
#define REG_TORQUE 40
#define REG_GOAL_POS 42
#define REG_PRESENT 56
#define PRESENT_LEN 15 /* 56..70 */
#define CURRENT_OFFSET (69 - REG_PRESENT)
#define CURRENT_LSB_MA 6.5f
#define RX_TIMEOUT_US 1500

static uint8_t checksum(const uint8_t *p, size_t n) { /* over ID..last param */
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s += p[i];
    return (uint8_t)~s;
}

static esp_err_t send_packet(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t np) {
    uint8_t buf[16 + 8 * ROBOT_NUM_JOINTS];
    if ((size_t)np + 6 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = (uint8_t)(np + 2);
    buf[4] = instr;
    memcpy(&buf[5], params, np);
    buf[5 + np] = checksum(&buf[2], (size_t)np + 3);
    return servo_uart_write(buf, (size_t)np + 6);
}

static int16_t sign_mag(uint16_t v, int bit) { return (v & (1u << bit)) ? -(int16_t)(v & ((1u << bit) - 1)) : (int16_t)v; }

/* Returns true and fills data[] when a valid reply from `id` was received. */
static bool read_regs(uint8_t id, uint8_t addr, uint8_t len, uint8_t *data) {
    uint8_t p[2] = {addr, len};
    servo_uart_flush_rx();
    if (send_packet(id, 0x02 /*READ*/, p, 2) != ESP_OK) return false;
    uint8_t r[6 + PRESENT_LEN];
    size_t want = (size_t)len + 6;
    if (servo_uart_read(r, want, RX_TIMEOUT_US) != want) return false;
    if (r[0] != 0xFF || r[1] != 0xFF || r[2] != id || r[3] != len + 2 || r[4] != 0) return false;
    if (r[5 + len] != checksum(&r[2], (size_t)len + 3)) return false;
    memcpy(data, &r[5], len);
    return true;
}

static esp_err_t feetech_read_all(servo_driver_t *d, servo_feedback_t fb[ROBOT_NUM_JOINTS]) {
    (void)d;
    uint8_t b[PRESENT_LEN];
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        fb[i].valid = read_regs(ROBOT_JOINT_ID[i], REG_PRESENT, PRESENT_LEN, b);
        if (!fb[i].valid) continue;
        uint16_t pos = (uint16_t)(b[0] | (b[1] << 8));
        uint16_t spd = (uint16_t)(b[2] | (b[3] << 8));
        uint16_t cur = (uint16_t)(b[CURRENT_OFFSET] | (b[CURRENT_OFFSET + 1] << 8));
        uint16_t load = (uint16_t)(b[4] | (b[5] << 8));
        fb[i].pos_rad = ((int)(pos & 0x0FFF) - CENTRE_TICK) * (TWO_PI / TICKS_PER_REV);
        fb[i].vel_rad_s = sign_mag(spd, 15) * (TWO_PI / TICKS_PER_REV);
        /* current register is unsigned magnitude; borrow the direction bit from load (bit10). */
        float ma = (float)(cur & 0x7FFF) * CURRENT_LSB_MA;
        fb[i].current_ma = (int16_t)fminf(32767.0f, (load & (1u << 10)) ? -ma : ma);
    }
    return ESP_OK;
}

static esp_err_t feetech_write_positions(servo_driver_t *d, const float raw_rad[ROBOT_NUM_JOINTS]) {
    (void)d;
    /* SYNC WRITE (0x83): params = addr, bytes-per-servo, then {id, data...} per servo.
     * data = goal position(2) + time(2)=0 + speed(2)=0 (0 == servo's max speed; the
     * firmware slew limiter already bounds how far a target can jump per tick). */
    uint8_t p[2 + 7 * ROBOT_NUM_JOINTS];
    size_t n = 0;
    p[n++] = REG_GOAL_POS;
    p[n++] = 6;
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        int t = CENTRE_TICK + (int)lroundf(raw_rad[i] * (TICKS_PER_REV / TWO_PI));
        if (t < 0) t = 0;
        if (t > 4095) t = 4095;
        p[n++] = ROBOT_JOINT_ID[i];
        p[n++] = (uint8_t)(t & 0xFF);
        p[n++] = (uint8_t)(t >> 8);
        memset(&p[n], 0, 4);
        n += 4;
    }
    return send_packet(0xFE, 0x83, p, (uint8_t)n);
}

static esp_err_t feetech_set_torque(servo_driver_t *d, bool enable) {
    (void)d;
    uint8_t p[2] = {REG_TORQUE, enable ? 1 : 0};
    return send_packet(0xFE, 0x03 /*WRITE*/, p, 2); /* broadcast: no reply */
}

static servo_driver_t s_driver = {
    .name = "feetech_sts", .read_all = feetech_read_all,
    .write_positions = feetech_write_positions, .set_torque = feetech_set_torque,
};
servo_driver_t *servo_feetech_sts_driver(void) { return &s_driver; }
