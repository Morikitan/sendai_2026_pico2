add_library(main_to_sub main_to_sub.cpp)

target_include_directories(main_to_sub PRIVATE ../display)
target_include_directories(main_to_sub PRIVATE ../gyro)
target_include_directories(main_to_sub PRIVATE ../motor)
target_include_directories(main_to_sub PRIVATE ../other_sensor)
target_include_directories(main_to_sub PRIVATE ../others)
target_include_directories(main_to_sub PRIVATE ../servo)
target_include_directories(main_to_sub PRIVATE ../tof)

target_link_libraries(main_to_sub pico_stdlib hardware_uart)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(main_to_sub PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)