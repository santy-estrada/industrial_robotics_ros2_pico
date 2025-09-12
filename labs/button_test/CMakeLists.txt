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

project(button_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, v ersion 0.1

add_executable(button_test button_test.c )

pico_set_program_name(button_test "button_test")
pico_set_program_version(button_test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(button_test 0)
pico_enable_stdio_usb(button_test 1)

# Add the standard library to the build
# This includes the standard C and C++ libraries
# This also includes the pico_stdlib library
# *hardware_gpio* is required for the GPIO functions
target_link_libraries(button_test
        pico_stdlib
        hardware_gpio
        )

# Add the standard include files to the build
target_include_directories(button_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(button_test)