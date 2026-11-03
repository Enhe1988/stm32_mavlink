cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)



# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME fly_control_stm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Project uses C11 and the startup assembly only.

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    application/Aircraft_ctrl_task.c
    application/INS_task.c
    # MAVLink 接收(DMA+IDLE,双端口:USART3 遥控/上位机 + USART2 光流)
    application/mavlink_port.c
    application/mavlink_rc.c
    application/mavlink_flow.c
    application/mavlink_app.c
    application/mavlink_telemetry.c
    algorithm/optical_flow.c
    algorithm/pid.c
    algorithm/filter.c
    algorithm/aircraft_mixer.c
    algorithm/attitude_control.c
    bsp/bmi08a.c
    bsp/bmi08g.c
    bsp/bmi08xa.c
    bsp/bmi088_anymotiona.c
    bsp/bmi088_mma.c
    bsp/common.c
    bsp/spl06.c
    # IMU 纯 C：统一接口、共享 ICM 采集、芯片配置与公共处理
    bsp/imu/imu_api.c
    bsp/imu/icm_acq.c
    bsp/imu/icm45686.c
    bsp/imu/icm42688p.c
    bsp/imu/bmi088.c
    bsp/bsp_platform.c
    bsp/imu/imu_frame.c
    bsp/imu/imu_calib.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    application
    algorithm
    bsp
    bsp/imu
    Third_party
    Third_party/mavlink_c/common
    Third_party/mavlink_c
    Third_party/mavlink_c/minimal
    Third_party/mavlink_c/standard
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
