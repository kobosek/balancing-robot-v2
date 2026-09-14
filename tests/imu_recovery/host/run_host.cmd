@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..\..") do set "ROOT=%%~fI"

if not defined IDF_PATH if defined ESP_IDF_PATH set "IDF_PATH=%ESP_IDF_PATH%"
if not defined IDF_PATH set "IDF_PATH=C:\Users\Karol\esp\v5.3\esp-idf"
if not exist "%IDF_PATH%\components\unity\unity\src\unity.h" (
    echo ESP-IDF Unity was not found under "%IDF_PATH%".
    echo Set IDF_PATH or ESP_IDF_PATH to an ESP-IDF checkout and retry.
    exit /b 2
)

where cl >nul 2>&1
if errorlevel 1 (
    set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
    if not exist "!VCVARS!" (
        echo cl.exe is not available and the default Visual Studio environment was not found.
        exit /b 2
    )
    call "!VCVARS!" >nul
    if errorlevel 1 exit /b 2
)

set "BUILD_DIR=%TEMP%\balancing_robot_imu_host_%RANDOM%_%RANDOM%"
mkdir "%BUILD_DIR%" >nul 2>&1
if errorlevel 1 exit /b 2

rem Resolve the ESP-IDF header once, then generate the wrapper in the
rem temporary build directory. This keeps the checked-in runner portable.
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "$template=Get-Content -Raw -LiteralPath '%SCRIPT_DIR%unity_adapter.in';" ^
    "$idf=((Get-Item -LiteralPath $env:IDF_PATH).FullName -replace '\\','/');" ^
    "$header=$idf + '/components/unity/unity/src/unity.h';" ^
    "$template.Replace('@IDF_UNITY_HEADER@',$header) | Set-Content -LiteralPath '%BUILD_DIR%\unity.h' -Encoding ascii"
if errorlevel 1 (
    echo Could not generate the Unity adapter.
    exit /b 2
)

pushd "%BUILD_DIR%"
set "UNITY_SRC=%IDF_PATH%\components\unity\unity\src\unity.c"
set "CJSON_SRC=%IDF_PATH%\components\json\cJSON\cJSON.c"

echo [host] control and strategy regressions
cl /nologo /std:c++20 /D_HAS_EXCEPTIONS=0 /D_USE_MATH_DEFINES /DUNITY_INCLUDE_DOUBLE ^
    /FI"%SCRIPT_DIR%esp_err.h" ^
    /I"%BUILD_DIR%" /I"%SCRIPT_DIR%." ^
    /I"%ROOT%\main\core\include" /I"%ROOT%\main\core\events" ^
    /I"%ROOT%\main\algorithms\include" /I"%ROOT%\main\control_math\include" ^
    /I"%ROOT%\main\services\configuration_service\include" ^
    "%SCRIPT_DIR%unity_main.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_control_math.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_longitudinal_strategy.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_nested_pid_reference.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_strategy_selection.cpp" ^
    "%ROOT%\main\algorithms\BalancingAlgorithm.cpp" ^
    "%ROOT%\main\algorithms\LongitudinalCascadeBalanceStrategy.cpp" ^
    "%ROOT%\main\algorithms\LongitudinalMotionProfile.cpp" ^
    "%ROOT%\main\algorithms\NestedPidBalanceStrategy.cpp" ^
    "%ROOT%\main\algorithms\PIDController.cpp" ^
    "%ROOT%\main\control_math\src\PidCore.cpp" ^
    "%ROOT%\main\control_math\src\WheelVelocityController.cpp" ^
    "%ROOT%\main\core\EventBus.cpp" ^
    "%UNITY_SRC%" /Fe:control_host.exe
if errorlevel 1 goto :failed
control_host.exe
if errorlevel 1 goto :failed

echo [host] configuration and migration regressions
cl /nologo /std:c++20 /D_HAS_EXCEPTIONS=0 /DUNITY_INCLUDE_DOUBLE ^
    /I"%BUILD_DIR%" /I"%SCRIPT_DIR%." ^
    /I"%ROOT%\main\core\include" /I"%ROOT%\main\core\events" ^
    /I"%ROOT%\main\algorithms\include" ^
    /I"%ROOT%\main\services\configuration_service\include" ^
    /I"%ROOT%\main\services\imu_service\include" ^
    /I"%IDF_PATH%\components\json\cJSON" ^
    "%SCRIPT_DIR%unity_main.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_config_migration.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_configuration_operations.cpp" ^
    "%ROOT%\main\services\configuration_service\ConfigurationService.cpp" ^
    "%ROOT%\main\services\configuration_service\ConfigChangePublisher.cpp" ^
    "%ROOT%\main\services\configuration_service\ConfigValidator.cpp" ^
    "%ROOT%\main\services\configuration_service\JsonConfigParser.cpp" ^
    "%ROOT%\main\services\configuration_service\JsonConfigSectionCodecs.cpp" ^
    "%ROOT%\main\core\EventBus.cpp" ^
    "%CJSON_SRC%" "%UNITY_SRC%" /Fe:config_host.exe
if errorlevel 1 goto :failed
config_host.exe
if errorlevel 1 goto :failed

echo [host] operation-gate regressions
cl /nologo /std:c++20 /D_HAS_EXCEPTIONS=0 /DUNITY_INCLUDE_DOUBLE ^
    /I"%BUILD_DIR%" /I"%SCRIPT_DIR%." /I"%ROOT%\main\core\include" ^
    "%SCRIPT_DIR%unity_main.cpp" ^
    "%ROOT%\tests\imu_recovery\main\test_operation_gate.cpp" ^
    "%UNITY_SRC%" /Fe:gate_host.exe
if errorlevel 1 goto :failed
gate_host.exe
if errorlevel 1 goto :failed

popd
echo Host regressions passed. Temporary build: %BUILD_DIR%
exit /b 0

:failed
popd
echo Host regression runner failed. Temporary build: %BUILD_DIR%
exit /b 1
