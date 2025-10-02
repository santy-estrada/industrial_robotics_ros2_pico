# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(robotics_classes LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Add the motor classes as a library
add_library(motor_lib
    motor/Motor.cpp
    motor/PrecisionMotor.cpp
    motor/ServoMotor.cpp
)

# Add the sensor classes as a library
add_library(sensor_lib
    sensor/TEMT6000.cpp
    sensor/LimitSwitch.cpp
    sensor/MPS20N0040D.cpp
    sensor/mpu9250.cpp
    sensor/madgwick_filter.cpp
    sensor/ultrasonic_sensor.cpp
    sensor/bmi160.cpp
)

# Add the robot classes as a library
add_library(robot_lib
    robot/DiffDrive.cpp
)



# Include directories for the libraries
target_include_directories(motor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(sensor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(robot_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)


# Link hardware libraries to the motor library
target_link_libraries(motor_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
)

# Link hardware libraries to the sensor library
target_link_libraries(sensor_lib
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_spi
)

# Link hardware libraries to the robot library
target_link_libraries(robot_lib
    motor_lib
    sensor_lib
    pico_stdlib
    hardware_gpio
    hardware_i2c
)


# Enable C++17 for std::map support
target_compile_features(motor_lib PUBLIC cxx_std_17)

# Create differential drive executable
add_executable(diff_app
    diff_app.cpp
)

# Link the motor and sensor libraries to the differential drive app
target_link_libraries(diff_app
    motor_lib
    sensor_lib
    robot_lib
    pico_stdlib
)

# Enable C++17 for the differential drive app
target_compile_features(diff_app PUBLIC cxx_std_17)

# Configure Pico for differential drive app
pico_enable_stdio_usb(diff_app 1)
pico_enable_stdio_uart(diff_app 0)

# Generate UF2 for differential drive app
pico_add_extra_outputs(diff_app)
