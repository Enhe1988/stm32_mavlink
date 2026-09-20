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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME fly_control_stm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

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
    algorithm/pid.c
    algorithm/filter.c
    bsp/bmi08a.c
    bsp/bmi08g.c
    bsp/bmi08xa.c
    bsp/bmi088_anymotiona.c
    bsp/bmi088_mma.c
    bsp/common.c
    bsp/spl06.c
    # IMU 抽象层(我们写)
    bsp/imu/imu_api.cpp
    bsp/imu/Icm45686.cpp
    bsp/imu/Icm42688p.cpp
    bsp/imu/Bmi088.cpp
    bsp/imu/icm_platform.c
    bsp/imu/imu_frame.c
    bsp/imu/imu_calib.c
    bsp/imu/imu_selftest.cpp
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

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
