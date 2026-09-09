set(PRODUCTION "${CMAKE_CURRENT_LIST_DIR}/../../../main")
idf_component_register(
    SRCS "test_main.cpp" "test_motion_safety.cpp" "fakes.cpp" "sensor_fakes.cpp"
         "test_control_timing.cpp" "${PRODUCTION}/algorithms/PIDController.cpp"
         "test_encoder.cpp" "pcnt_fakes.cpp" "test_sensor_frame.cpp"
         "${PRODUCTION}/services/EncoderService.cpp"
         "test_fifo_stream.cpp" "test_imu_recovery.cpp" "test_config_migration.cpp" "test_orientation_quality.cpp"
         "${PRODUCTION}/services/MotorService.cpp"
         "${PRODUCTION}/core/StateManagerPolicy.cpp"
         "${PRODUCTION}/core/StateManager.cpp"
         "${PRODUCTION}/core/EventBus.cpp"
         "${PRODUCTION}/core/ControlEventDispatcher.cpp"
         "${PRODUCTION}/algorithms/BalanceMonitor.cpp"
         "${PRODUCTION}/services/imu_service/IMUService.cpp"
         "${PRODUCTION}/services/imu_service/IMUTask.cpp"
         "${PRODUCTION}/services/imu_service/OrientationEstimator.cpp"
         "${PRODUCTION}/services/imu_service/FIFOProcessor.cpp"
         "${PRODUCTION}/services/imu_service/IMUCalibration.cpp"
         "${PRODUCTION}/services/imu_service/mpu6050.cpp"
         "${PRODUCTION}/services/imu_service/MPU6050HardwareController.cpp"
         "${PRODUCTION}/services/configuration_service/JsonConfigParser.cpp"
         "${PRODUCTION}/services/configuration_service/JsonConfigSectionCodecs.cpp"
         "${PRODUCTION}/services/configuration_service/ConfigValidator.cpp"
    INCLUDE_DIRS "." "${PRODUCTION}/core/include" "${PRODUCTION}/core/events"
                 "${PRODUCTION}/services/include" "${PRODUCTION}/drivers/include"
                 "${PRODUCTION}/services/configuration_service/include"
                 "${PRODUCTION}/services/imu_service/include" "${PRODUCTION}/web/include"
                 "${PRODUCTION}/algorithms/include" "${PRODUCTION}/tasks/include"
    REQUIRES unity driver esp_timer pthread json esp_adc
    WHOLE_ARCHIVE)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++2b)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_timer_get_time")
# Production I2C, IRQ and motor implementations are deliberately absent.
