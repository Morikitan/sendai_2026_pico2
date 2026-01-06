# lib/u8g2/CMakeLists.txt

cmake_minimum_required(VERSION 3.13)

# プロジェクト名（ここではライブラリとしてビルドするので project() は不要）

project(u8g2)

# 必要な .c ファイルを明示的に指定
set(U8G2_SOURCES
    csrc/u8g2_setup.c
    csrc/u8g2_d_setup.c
    csrc/u8g2_ll_hvline.c
    csrc/u8g2_font.c
    csrc/u8g2_fonts.c
    csrc/u8g2_line.c
    csrc/u8g2_d_memory.c
    csrc/u8g2_hvline.c
    csrc/u8g2_bitmap.c
    csrc/u8g2_selection_list.c
    csrc/u8g2_buffer.c
    csrc/u8g2_circle.c
    csrc/u8g2_kerning.c
    csrc/u8g2_intersection.c

    csrc/u8x8_byte.c
    csrc/u8x8_8x8.c
    csrc/u8x8_cad.c
    csrc/u8x8_capture.c
    csrc/u8x8_d_ssd1306_128x64_noname.c
    csrc/u8x8_d_ssd1309.c
    csrc/u8x8_debounce.c
    csrc/u8x8_display.c
    csrc/u8x8_fonts.c
    csrc/u8x8_gpio.c
    csrc/u8x8_input_value.c
    csrc/u8x8_message.c
    csrc/u8x8_selection_list.c
    csrc/u8x8_setup.c
    csrc/u8x8_string.c
    csrc/u8x8_u8toa.c
    csrc/u8x8_u16toa.c
)

file(GLOB U8G2_HEADERS
    csrc/*.h
)

# ライブラリを静的に作成
add_library(u8g2 STATIC ${U8G2_SOURCES} ${U8G2_HEADERS})

# ヘッダーファイルのインクルードディレクトリを公開
target_include_directories(u8g2 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/csrc
)
