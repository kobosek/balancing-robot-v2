# main/CMakeLists.txt

idf_component_register(SRCS # --- Tasks ---
                            "tasks/ControlTask.cpp"
                            # --- Core ---
                            "core/EventBus.cpp"
                            "core/StateManager.cpp"
                            "core/RobotController.cpp" 
                            "core/Application.cpp"
                            # --- Configuration Service ---
                            "services/configuration_service/SPIFFSStorageService.cpp"
                            "services/configuration_service/JsonConfigParser.cpp"
                            "services/configuration_service/ConfigurationService.cpp"
                            # --- IMU Service---
                            "services/imu_service/IMUService.cpp"
                            "services/imu_service/IMUCalibration.cpp"
                            "services/imu_service/IMUHealthMonitor.cpp"
                            "services/imu_service/OrientationEstimator.cpp" # Refactored Estimator
                            "services/imu_service/mpu6050.cpp"
                            "services/imu_service/FIFOTask.cpp"
                            "services/imu_service/HealthMonitorTask.cpp"
                            "services/imu_service/FIFOProcessor.cpp"
                            # --- Battery Service 
                            "services/battery_service/BatteryService.cpp"
                            "services/battery_service/BatteryTask.cpp"
                            # --- Services ---
                            "services/WiFiManager.cpp"
                            "services/EncoderService.cpp"
                            "services/MotorService.cpp"
                            # --- Web ---
                            "web/WebServer.cpp"
                            "web/StaticFileHandler.cpp"
                            "web/TelemetryHandler.cpp"
                            "web/ConfigApiHandler.cpp"
                            "web/CommandApiHandler.cpp"
                            "web/StateApiHandler.cpp"
                            # --- Algorithms ---
                            "algorithms/PIDController.cpp"
                            "algorithms/BalancingAlgorithm.cpp"
                            "algorithms/FallDetector.cpp"
                            "algorithms/CommandProcessor.cpp"
                            # --- Drivers ---
                            "drivers/MX1616H_HWDriver.cpp"
                            # --- Main Entry ---
                            "main.cpp"
                       INCLUDE_DIRS "."
                                    "core/include"
                                    "core/events"
                                    "services/include"
                                    "services/battery_service/include"
                                    "services/configuration_service/include"
                                    "services/imu_service/include"
                                    "web/include"
                                    "tasks/include"
                                    "drivers/include"
                                    "algorithms/include"
                       REQUIRES esp_wifi esp_event esp_netif esp_http_server nvs_flash
                                esp_timer spiffs driver json pthread esp_adc 
                       PRIV_REQUIRES json
                      )
