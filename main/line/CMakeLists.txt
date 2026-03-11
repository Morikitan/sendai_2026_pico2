add_executable(main_line main_line.cpp )

pico_set_program_name(main_line "main_line")
pico_set_program_version(main_line "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main_line 0)
pico_enable_stdio_usb(main_line 1)

target_include_directories(main_line PRIVATE ../../lib)

# Add the standard include files to the build
target_include_directories(main_line PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(main_line pico_stdlib hardware_i2c hardware_gpio)
target_link_libraries(main_line line main_to_line)

target_include_directories(main_line PRIVATE ../../lib/display/u8g2/csrc)

pico_add_extra_outputs(main_line)