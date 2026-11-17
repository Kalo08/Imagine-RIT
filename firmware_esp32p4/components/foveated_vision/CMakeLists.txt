idf_component_register(SRCS "foveated_vision.c" "motion_tracker.c" "camera_v4l2.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES generated
                       PRIV_REQUIRES esp_timer freertos log esp_video)
