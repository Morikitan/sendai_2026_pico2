add_library(other_sensor other_sensor.cpp)

target_include_directories(other_sensor PRIVATE ../display)

target_link_libraries(other_sensor pico_stdlib hardware_adc hardware_pwm hardware_i2c)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(other_sensor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)