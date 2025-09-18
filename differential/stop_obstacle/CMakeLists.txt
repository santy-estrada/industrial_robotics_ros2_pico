# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location

# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(stop_obstacle C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(stop_obstacle stop_obstacle.c )

pico_set_program_name(stop_obstacle "stop_obstacle")
pico_set_program_version(stop_obstacle "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(stop_obstacle 0)
pico_enable_stdio_usb(stop_obstacle 1)

# Add the standard library to the build
target_link_libraries(stop_obstacle
        pico_stdlib
        hardware_pwm
        hardware_irq
        hardware_gpio
        hardware_adc
        )

# Add the standard include files to the build
target_include_directories(stop_obstacle PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(stop_obstacle)

