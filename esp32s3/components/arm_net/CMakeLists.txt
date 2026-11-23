idf_component_register(SRCS "wifi_mgr.c"
                       INCLUDE_DIRS "include"
                       REQUIRES arm_config arm_link esp_wifi esp_netif esp_event lwip log)
