add_library(encoder encoder.cpp)

target_include_directories(encoder PRIVATE ../display)

target_link_libraries(encoder pico_stdlib)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(encoder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)