add_library(action action.cpp)

target_include_directories(action PRIVATE ../camera)
target_include_directories(action PRIVATE ../display)
target_include_directories(action PRIVATE ../gyro)
target_include_directories(action PRIVATE ../line)
target_include_directories(action PRIVATE ../main_to_line)
target_include_directories(action PRIVATE ../main_to_sub)
target_include_directories(action PRIVATE ../motor)
target_include_directories(action PRIVATE ../other_sensor)
target_include_directories(action PRIVATE ../others)
target_include_directories(action PRIVATE ../servo)

target_link_libraries(action pico_stdlib hardware_uart)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(action PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)