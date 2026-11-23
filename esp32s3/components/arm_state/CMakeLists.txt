idf_component_register(SRCS "arm_state.c"
                       INCLUDE_DIRS "include"
                       REQUIRES arm_config arm_ik arm_bus log nvs_flash)
