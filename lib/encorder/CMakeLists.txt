add_library(encorder encorder.cpp)

target_include_directories(encorder PRIVATE ../display)

target_link_libraries(encorder pico_stdlib)

#display.hppの中のu8g2ライブラリを有効にする
target_include_directories(encorder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../display/u8g2/csrc
)