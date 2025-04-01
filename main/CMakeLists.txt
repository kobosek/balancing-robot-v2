idf_component_register(SRCS  "PCNTEncoder.cpp" "Encoder.cpp" "MPU6050Manager.cpp" 
                            "RuntimeConfig.cpp"
                            "ComponentHandler.cpp"
                            "WiFiManager.cpp"
                            "WebServer.cpp"
                            "MotorDriver.cpp"                           
                            "PIDController.cpp"
                            "mpu6050.cpp"
                            "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi
                                esp_event
                                esp_netif
                                esp_http_server
                                nvs_flash
                                esp_timer
                                spiffs
                                driver
                                json
                                pthread)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/json/cJSON)

