idf_component_register(SRCS "usb_bridge.c"
                       INCLUDE_DIRS "include"
                       REQUIRES generated
                       PRIV_REQUIRES esp_timer freertos log)
