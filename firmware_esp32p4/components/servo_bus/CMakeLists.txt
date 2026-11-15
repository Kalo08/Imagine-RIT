idf_component_register(SRCS "servo_bus.c" "servo_uart.c" "servo_feetech.c" "servo_dynamixel.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES generated
                       PRIV_REQUIRES driver esp_timer esp_driver_uart esp_driver_gpio)
