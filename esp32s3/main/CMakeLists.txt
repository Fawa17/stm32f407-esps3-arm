idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES arm_config arm_ik arm_state arm_link arm_net arm_web nvs_flash esp_timer)
