add_library(display display.cpp)

# u8g2をサブディレクトリとして追加（u8g2/CMakeLists.txtが存在する場合）
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/u8g2 ${CMAKE_CURRENT_BINARY_DIR}/u8g2_build)

# u8g2のヘッダを検索パスに追加
target_include_directories(display PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}           # display.hpp
    ${CMAKE_CURRENT_LIST_DIR}/u8g2/csrc # ✅ ここをまとめて PUBLIC に
)

target_link_libraries(display u8g2 hardware_i2c pico_stdlib hardware_gpio)

#target_include_directories(display PRIVATE u8g2/csrc)