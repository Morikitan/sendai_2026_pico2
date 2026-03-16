add_executable(main_main main_main.cpp )

pico_set_program_name(main_main "main_main")
pico_set_program_version(main_main "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main_main 0)
pico_enable_stdio_usb(main_main 1)

target_include_directories(main_main PRIVATE ../../lib)

# Add the standard include files to the build
target_include_directories(main_main PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(main_main pico_stdlib hardware_i2c)
target_link_libraries(main_main motor others display main_to_line line)
target_link_libraries(main_main other_sensor main_to_sub gyro tof action camera)

target_include_directories(main_main PRIVATE ../../lib/display/u8g2/csrc)

pico_add_extra_outputs(main_main)