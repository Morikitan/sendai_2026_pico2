add_executable(main_sub main_sub.cpp )

pico_set_program_name(main_sub "main_sub")
pico_set_program_version(main_sub "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main_sub 0)
pico_enable_stdio_usb(main_sub 1)

target_include_directories(main_sub PRIVATE ../../lib)

# Add the standard include files to the build
target_include_directories(main_sub PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(main_sub pico_stdlib hardware_i2c)

target_include_directories(main_sub PRIVATE ../../lib/display/u8g2/csrc)

pico_add_extra_outputs(main_sub)