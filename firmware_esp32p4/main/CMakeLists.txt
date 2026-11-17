idf_component_register(SRCS "app_main.c" "board_display.c"
                       INCLUDE_DIRS "."
                       REQUIRES generated servo_bus imu_bno085 spinal_reflex foveated_vision face_ui usb_bridge
                       PRIV_REQUIRES esp_timer esp_system freertos log lvgl)
