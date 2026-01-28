add_library(tof tof.cpp)

add_subdirectory(VL53L0X)

target_include_directories(tof PRIVATE ../display)
target_include_directories(tof PRIVATE VL53L0X)

target_link_libraries(tof pico_stdlib VL53L0X_Lib hardware_i2c)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(tof PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)