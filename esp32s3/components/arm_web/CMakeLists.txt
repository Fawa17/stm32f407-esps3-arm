idf_component_register(SRCS "web_server.c"
                       INCLUDE_DIRS "include"
                       REQUIRES arm_config arm_ik arm_state arm_link arm_net esp_http_server app_update log
                       EMBED_TXTFILES "www/index.html")
