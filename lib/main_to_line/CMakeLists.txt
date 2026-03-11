add_library(main_to_line main_to_line.cpp)

target_include_directories(main_to_line PRIVATE ../display)
target_include_directories(main_to_line PRIVATE ../line)

target_link_libraries(main_to_line pico_stdlib hardware_pio)

pico_generate_pio_header(main_to_line ${CMAKE_CURRENT_LIST_DIR}/picoPioUart.pio)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(main_to_line PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)