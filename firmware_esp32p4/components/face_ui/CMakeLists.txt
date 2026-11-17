idf_component_register(SRCS "face_ui.c"
                       INCLUDE_DIRS "include"
                       REQUIRES generated lvgl esp_lvgl_port
                       PRIV_REQUIRES freertos log)
