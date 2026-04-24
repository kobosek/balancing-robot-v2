#include "ApplicationEventWiring.hpp"

#include "ApplicationContext.hpp"
#include "BALANCE_AutoBalanceReady.hpp"
#include "BALANCE_FallDetected.hpp"
#include "BALANCE_MonitorModeChanged.hpp"
#include "BalancingAlgorithm.hpp"
#include "BalanceMonitor.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_ImuConfigUpdate.hpp"
#include "CONFIG_PidConfigUpdate.hpp"
#include "COMMAND_InputModeChanged.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "ConfigurationService.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "GUIDED_CalibrationFinished.hpp"
#include "GUIDED_CalibrationRunModeChanged.hpp"
#include "GuidedCalibrationService.hpp"
#include "IMU_AttachRequested.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_CommunicationError.hpp"
#include "IMU_GyroOffsetsUpdated.hpp"
#include "IMU_OrientationData.hpp"
#include "IMU_SystemPolicyChanged.hpp"
#include "IMUService.hpp"
#include "MOTION_TargetMovement.hpp"
#include "MOTOR_OutputEnabledChanged.hpp"
#include "MotorService.hpp"
#include "OTAService.hpp"
#include "OTA_UpdatePolicyChanged.hpp"
#include "PID_TuningFinished.hpp"
#include "PID_TuningRunModeChanged.hpp"
#include "PidTuningService.hpp"
#include "RobotController.hpp"
#include "StateManager.hpp"
#include "TELEMETRY_Snapshot.hpp"
#include "UI_CalibrateImu.hpp"
#include "UI_CancelGuidedCalibration.hpp"
#include "UI_CancelPidTuning.hpp"
#include "UI_DisableAutoBalancing.hpp"
#include "UI_DisableFallDetection.hpp"
#include "UI_DiscardPidTuning.hpp"
#include "UI_EnableAutoBalancing.hpp"
#include "UI_EnableFallDetection.hpp"
#include "UI_JoystickInput.hpp"
#include "UI_SavePidTuning.hpp"
#include "UI_StartBalancing.hpp"
#include "UI_StartGuidedCalibration.hpp"
#include "UI_StartPidTuning.hpp"
#include "UI_Stop.hpp"
#include "WebServer.hpp"

namespace {
template <typename T>
std::shared_ptr<EventHandler> asHandler(const std::shared_ptr<T>& handler)
{
    return std::static_pointer_cast<EventHandler>(handler);
}
}

esp_err_t ApplicationEventWiring::wire(const ApplicationContext& context) const
{
    EventBus& eventBus = context.eventBus();

    eventBus.subscribe<COMMAND_InputModeChanged,
                       UI_JoystickInput,
                       CONFIG_FullConfigUpdate>(asHandler(context.commandProcessorHandle()));
    eventBus.subscribe<CONFIG_FullConfigUpdate, CONFIG_PidConfigUpdate>(asHandler(context.balancingAlgorithmHandle()));
    eventBus.subscribe<IMU_OrientationData,
                       BALANCE_MonitorModeChanged,
                       CONFIG_FullConfigUpdate>(asHandler(context.balanceMonitorHandle()));
    eventBus.subscribe<MOTOR_OutputEnabledChanged>(asHandler(context.motorServiceHandle()));
    eventBus.subscribe<UI_StartPidTuning,
                       UI_CancelPidTuning,
                       UI_SavePidTuning,
                       UI_DiscardPidTuning,
                       PID_TuningRunModeChanged>(asHandler(context.pidTuningServiceHandle()));
    eventBus.subscribe<UI_StartGuidedCalibration,
                       UI_CancelGuidedCalibration,
                       GUIDED_CalibrationRunModeChanged,
                       CONFIG_FullConfigUpdate>(asHandler(context.guidedCalibrationServiceHandle()));
    eventBus.subscribe<CONFIG_FullConfigUpdate>(asHandler(context.batteryServiceHandle()));
    eventBus.subscribe<BALANCE_FallDetected,
                       BALANCE_AutoBalanceReady,
                       UI_StartBalancing,
                       UI_Stop,
                       UI_EnableAutoBalancing,
                       UI_DisableAutoBalancing,
                       UI_EnableFallDetection,
                       UI_DisableFallDetection,
                       BATTERY_StatusUpdate,
                       UI_CalibrateImu,
                       UI_StartPidTuning,
                       UI_CancelPidTuning,
                       PID_TuningFinished,
                       UI_StartGuidedCalibration,
                       UI_CancelGuidedCalibration,
                       GUIDED_CalibrationFinished,
                       IMU_CalibrationCompleted,
                       IMU_CalibrationRequestRejected,
                       IMU_CommunicationError,
                       IMU_AvailabilityChanged,
                       CONFIG_FullConfigUpdate>(asHandler(context.stateManagerHandle()));
    eventBus.subscribe<IMU_GyroOffsetsUpdated>(asHandler(context.configServiceHandle()));
    eventBus.subscribe<CONFIG_FullConfigUpdate,
                       CONFIG_ImuConfigUpdate,
                       IMU_CalibrationRequest,
                       IMU_AttachRequested,
                       IMU_SystemPolicyChanged>(asHandler(context.imuServiceHandle()));
    eventBus.subscribe<MOTION_TargetMovement,
                       CONTROL_RunModeChanged>(asHandler(context.robotControllerHandle()));
    eventBus.subscribe<OTA_UpdatePolicyChanged>(asHandler(context.otaServiceHandle()));
    eventBus.subscribe<CONFIG_FullConfigUpdate, TELEMETRY_Snapshot>(asHandler(context.webServerHandle()));

    return ESP_OK;
}
