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

project(analog_io C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(analog_io analog_io.c )

pico_set_program_name(analog_io "analog_io")
pico_set_program_version(analog_io "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(analog_io 0)
pico_enable_stdio_usb(analog_io 1)

# Add the standard library to the build
# target_link_libraries(analog_io pico_stdlib)
# *hardware_adc* is required for analog input
# *hardware_pwm* is required for analog output
target_link_libraries(analog_io
        pico_stdlib
        hardware_adc
        hardware_pwm
)

# Add the standard include files to the build
target_include_directories(analog_io PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(analog_io)