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
set(CMAKE_PROJECT_NAME MoFingerTop_magnet_uart)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

option(BOARD_ENABLE_USART1_IN "Build finger-pulp magnet firmware with USART1 IN port" OFF)
option(BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND "Use TIM2 1000 Hz fixed frame send and MLX burst mode" ON)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/com/Src/com_crc.c
    Core/bsp/Src/board_config.c
    Core/bsp/Src/bsp_i2c.c
    Core/bsp/Src/bsp_time.c
    Core/bsp/Src/bsp_uart_in.c
    Core/bsp/Src/bsp_uart_dma.c
    Core/bsp/Src/h503_flash.c
    Core/dri/Src/mlx90393.c
    Core/dri/Src/lsm6dsow.c
    Core/mid/Src/sensor_protocol.c
    Core/app/Src/sensor_config.c
    Core/app/Src/sensor_app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/com/Inc
    Core/bsp/Inc
    Core/dri/Inc
    Core/mid/Inc
    Core/app/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
