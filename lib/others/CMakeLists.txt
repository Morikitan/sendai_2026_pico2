add_library(others others.cpp)

target_include_directories(others PRIVATE ../display)

target_link_libraries(others pico_stdlib)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(others PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)