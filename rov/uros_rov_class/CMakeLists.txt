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

# Create motor library
add_library(motor_lib 
    motor/Motor.cpp
)
target_include_directories(motor_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(motor_lib pico_stdlib hardware_pwm)

# Create sensor library
add_library(sensor_lib
    sensor/LimitSwitch.cpp
    sensor/SOIL_MOIST.cpp
    sensor/MPS20N0040D.cpp
    sensor/TEMT6000.cpp
)
target_include_directories(sensor_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(sensor_lib pico_stdlib hardware_adc)

# Create ROV library
add_library(rov_lib
    rov/ROV.cpp
)
target_include_directories(rov_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(rov_lib motor_lib sensor_lib)

# ROV micro-ROS executable
add_executable(uros_rov_obj
				uros_rov_obj.cpp
				pico_uart_transport.c ) 

pico_set_program_name(uros_rov_obj "uros_rov_obj")
pico_set_program_version(uros_rov_obj "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(uros_rov_obj 0)
pico_enable_stdio_usb(uros_rov_obj 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0) # <<<<------****COPIAR**** 
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0) # <<<<------****COPIAR**** 
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0) # <<<<------****COPIAR**** 

# Add the standard library to the build
target_link_libraries(uros_rov_obj
        pico_stdlib
		hardware_adc
		hardware_pwm
		motor_lib
		sensor_lib
		rov_lib
		microros) 

target_include_directories(uros_rov_obj PUBLIC
    libmicroros/include
) 
		
# Add the standard include files to the build
target_include_directories(uros_rov_obj PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/..
) 

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections") # <<<<------****COPIAR**** 
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections") # <<<<------****COPIAR**** 

pico_add_extra_outputs(uros_rov_obj)

