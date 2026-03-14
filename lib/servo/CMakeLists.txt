add_library(servo servo.cpp)

target_include_directories(servo PRIVATE ../display)
target_include_directories(servo PRIVATE ../main_to_sub)

target_link_libraries(servo pico_stdlib hardware_pwm)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(servo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)