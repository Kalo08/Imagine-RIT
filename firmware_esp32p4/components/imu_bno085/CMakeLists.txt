idf_component_register(SRCS "imu_bno085.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer freertos log)
