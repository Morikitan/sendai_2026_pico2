add_library(motor motor.cpp)

target_include_directories(motor PRIVATE ../display)

target_link_libraries(motor pico_stdlib)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(motor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)