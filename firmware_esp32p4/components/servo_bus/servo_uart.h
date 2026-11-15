/* Private half-duplex UART helper shared by the servo drivers. */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t servo_uart_init(void);
void servo_uart_flush_rx(void);
/* Transmit and block until the last bit has left the wire (so the bus direction flips back). */
esp_err_t servo_uart_write(const uint8_t *data, size_t len);
/* Read exactly `len` bytes within timeout_us; returns bytes actually read. */
size_t servo_uart_read(uint8_t *buf, size_t len, uint32_t timeout_us);
