add_library(line line.cpp)

target_include_directories(line PRIVATE ../display)

target_link_libraries(line pico_stdlib)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(line PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)