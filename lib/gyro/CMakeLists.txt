add_library(gyro gyro.cpp)

target_include_directories(gyro PRIVATE ../display)
target_include_directories(gyro PRIVATE ../others)

target_link_libraries(gyro pico_stdlib hardware_i2c hardware_gpio)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(gyro PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)