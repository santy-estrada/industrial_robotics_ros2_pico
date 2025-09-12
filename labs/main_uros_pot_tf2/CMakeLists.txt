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

project(main_uros_pot_tf2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

link_directories(libmicroros) # <<<<------****COPIAR**** 

add_executable(main_uros_pot_tf2 
				main_uros_pot_tf2.c
				pico_uart_transport.c ) # <<<<------****COPIAR**** 

pico_set_program_name(main_uros_pot_tf2 "main_uros_pot_tf2")
pico_set_program_version(main_uros_pot_tf2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main_uros_pot_tf2 0)
pico_enable_stdio_usb(main_uros_pot_tf2 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0) # <<<<------****COPIAR**** 
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0) # <<<<------****COPIAR**** 
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0) # <<<<------****COPIAR**** 

# Add the standard library to the build
target_link_libraries(main_uros_pot_tf2
        pico_stdlib
		hardware_adc
		hardware_pwm
		microros) # <<<<------****COPIAR**** 

target_include_directories(main_uros_pot_tf2 PUBLIC
    libmicroros/include
) # <<<<------****COPIAR**** 
		
# Add the standard include files to the build
target_include_directories(main_uros_pot_tf2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
) # <<<<------****COPIAR**** 

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections") # <<<<------****COPIAR**** 
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections") # <<<<------****COPIAR**** 

pico_add_extra_outputs(main_uros_pot_tf2)

