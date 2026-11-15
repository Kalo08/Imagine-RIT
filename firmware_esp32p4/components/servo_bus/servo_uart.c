#include "servo_uart.h"
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "robot_protocol.h"
#include "sdkconfig.h"

#define PORT ((uart_port_t)CONFIG_SERVO_BUS_UART_NUM)

esp_err_t servo_uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = ROBOT_BUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(PORT, 1024, 1024, 0, NULL, 0);
    if (e != ESP_OK) return e;
    if ((e = uart_param_config(PORT, &cfg)) != ESP_OK) return e;
    if ((e = uart_set_pin(PORT, CONFIG_SERVO_BUS_TX_GPIO, CONFIG_SERVO_BUS_RX_GPIO,
                          CONFIG_SERVO_BUS_DIR_GPIO, UART_PIN_NO_CHANGE)) != ESP_OK) return e;
    /* RS485 half-duplex mode drives the RTS pin high while transmitting: wire it to the
     * direction/enable pin of the tri-state buffer that joins TX and RX onto the single
     * servo data line. */
    if ((e = uart_set_mode(PORT, UART_MODE_RS485_HALF_DUPLEX)) != ESP_OK) return e;
    return uart_set_rx_timeout(PORT, 1);
}

void servo_uart_flush_rx(void) { uart_flush_input(PORT); }

esp_err_t servo_uart_write(const uint8_t *data, size_t len) {
    if (uart_write_bytes(PORT, data, len) < 0) return ESP_FAIL;
    return uart_wait_tx_done(PORT, pdMS_TO_TICKS(5));
}

size_t servo_uart_read(uint8_t *buf, size_t len, uint32_t timeout_us) {
    size_t got = 0;
    int64_t end = esp_timer_get_time() + (int64_t)timeout_us;
    while (got < len) {
        int64_t left = end - esp_timer_get_time();
        if (left <= 0) break;
        /* uart_read_bytes takes ticks; with a 1 kHz tick, poll in 1-tick slices and
         * rely on the RX FIFO for sub-tick timing. ticks==0 returns immediately. */
        TickType_t ticks = left > 1500 ? 1 : 0;
        int r = uart_read_bytes(PORT, buf + got, len - got, ticks);
        if (r > 0) got += (size_t)r;
    }
    return got;
}
