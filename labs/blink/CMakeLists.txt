#  Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# CMAKE System Information
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Blink2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Be carful with the 2nd parameter it is the name of the source file (be sure the name is exactly the same)
add_executable(Blink2 Blink2.c ) 

pico_set_program_name(Blink2 "Blink2")
pico_set_program_version(Blink2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
# this allows you to see the output of printf statements in the terminal
pico_enable_stdio_uart(Blink2 0)
pico_enable_stdio_usb(Blink2 1)

# Add the standard library to the build
# This includes the standard C and C++ libraries
target_link_libraries(Blink2
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Blink2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Blink2)