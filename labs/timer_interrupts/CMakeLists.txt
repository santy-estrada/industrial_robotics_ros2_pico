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

project(timer_interrupts C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(timer_interrupts timer_interrupts.c )

pico_set_program_name(timer_interrupts "timer_interrupts")
pico_set_program_version(timer_interrupts "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(timer_interrupts 0)
pico_enable_stdio_usb(timer_interrupts 1)

# Add the standard library to the build
# target_link_libraries(timer_interrupts pico_stdlib)
# Add any user requested libraries
### Add the hardware_timer library to the build
### Add the hardware_irq library to the build
### Add the hardware_gpio library to the build
target_link_libraries(timer_interrupts
        pico_stdlib
        hardware_timer
        hardware_irq
        hardware_gpio
        )

# Add the standard include files to the build
target_include_directories(timer_interrupts PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# extra outputs for generation of binary files
pico_add_extra_outputs(timer_interrupts)