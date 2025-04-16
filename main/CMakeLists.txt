# main/CMakeLists.txt

idf_component_register(SRCS # --- Tasks ---
                            "tasks/ControlTask.cpp"
                            "tasks/BatteryTask.cpp"
                            "tasks/IMUTasks.cpp"          # Contains both FIFO and Health tasks now
                            # --- Core ---
                            "core/EventBus.cpp"
                            "core/StateManager.cpp"
                            "core/RobotController.cpp"      # Facade
                            "core/Application.cpp"
                            # --- Config ---
                            "config/SPIFFSStorageService.cpp"
                            "config/JsonConfigParser.cpp"
                            "config/ConfigurationService.cpp"
                            # --- Services ---
                            "services/WiFiManager.cpp"
                            "services/IMUService.cpp"             # Refactored Service
                            "services/IMUCalibrationService.cpp"  # New Service
                            "services/IMUHealthMonitor.cpp"       # New Service
                            "services/EncoderService.cpp"
                            "services/MotorService.cpp"
                            "services/BatteryService.cpp"
                            # --- Web ---
                            "web/WebServer.cpp"
                            "web/StaticFileHandler.cpp"
                            "web/TelemetryHandler.cpp"
                            "web/ConfigApiHandler.cpp"
                            "web/CommandApiHandler.cpp"
                            "web/StateApiHandler.cpp"
                            # --- Algorithms ---
                            "algorithms/PIDController.cpp"
                            "algorithms/OrientationEstimator.cpp" # Refactored Estimator
                            "algorithms/BalancingAlgorithm.cpp"
                            "algorithms/FallDetector.cpp"
                            "algorithms/CommandProcessor.cpp"
                            # --- Drivers ---
                            "drivers/mpu6050.cpp"                 # Refactored Driver
                            "drivers/MX1616H_HWDriver.cpp"
                            # --- Main Entry ---
                            "main.cpp"
                       INCLUDE_DIRS "."
                                    "tasks/include"
                                    "algorithms/include"
                                    "config/include"
                                    "core/include"
                                    "drivers/include"
                                    "events"
                                    "services/include" # Added service include dir
                                    "web/include"
                       REQUIRES esp_wifi esp_event esp_netif esp_http_server nvs_flash
                                esp_timer spiffs driver json pthread esp_adc 
                       PRIV_REQUIRES json
                      )

