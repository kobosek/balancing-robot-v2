cmake_minimum_required(VERSION 3.16)

project(control_math LANGUAGES CXX)

add_library(control_math STATIC
    src/PidCore.cpp
    src/WheelVelocityController.cpp
)

target_compile_features(control_math PUBLIC cxx_std_17)
target_include_directories(control_math PUBLIC include)

add_executable(control_math_example examples/basic_usage.cpp)
target_link_libraries(control_math_example PRIVATE control_math)
