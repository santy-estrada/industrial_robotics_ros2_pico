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

project(mv_joints C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mv_joints mv_joints.c pwm/pwm.c motor/motor.c)

pico_set_program_name(mv_joints "mv_joints")
pico_set_program_version(mv_joints "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mv_joints 0)
pico_enable_stdio_usb(mv_joints 1)

# Add the standard library to the build
target_link_libraries(mv_joints
        pico_stdlib
        hardware_pwm
        hardware_gpio
        hardware_adc
        hardware_irq
        )

# Add the standard include files to the build
target_include_directories(mv_joints PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(mv_joints)