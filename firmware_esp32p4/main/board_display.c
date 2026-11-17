/*
 * The one board-specific seam for the round MIPI-DSI LCD: return an initialised LVGL 9 display.
 *
 * On a board with an ESP-BSP package (Waveshare ESP32-P4 boards provide one), add the BSP to
 * main/idf_component.yml and this file picks it up automatically. Without a BSP, implement
 * esp_lcd DSI panel bring-up + lvgl_port_add_disp_dsi() for your panel's controller here.
 */
#include "esp_log.h"
#include "lvgl.h"

#if __has_include("bsp/esp-bsp.h")
#include "bsp/esp-bsp.h"
lv_display_t *board_display_init(void) { return bsp_display_start(); }
#else
lv_display_t *board_display_init(void) {
    ESP_LOGE("board_display", "no BSP found (bsp/esp-bsp.h): add your board's BSP or panel init here");
    return NULL;
}
#endif
