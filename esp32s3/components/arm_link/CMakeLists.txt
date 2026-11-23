idf_component_register(SRCS "stm32_link.c"
                       INCLUDE_DIRS "include"
                       REQUIRES arm_config arm_bus arm_state driver esp_timer log)
