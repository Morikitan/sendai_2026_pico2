#include "camera.hpp"
#include "display.hpp"
#include "../config.hpp"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static constexpr uint32_t CAMERA_BAUD = 115200;
static constexpr int MAX_OBJECTS = 8;
static constexpr int RX_LINE_SIZE = 128;

struct ObjData {
    int index;
    float angle;
    int area;
    char orient;   // 'H' or 'V' for can
};

static ObjData red_objs[MAX_OBJECTS];
static ObjData blue_objs[MAX_OBJECTS];
static ObjData can_objs[MAX_OBJECTS];

static int red_count = 0;
static int blue_count = 0;
static int can_count = 0;
static uint32_t frame_id = 0;

static char rx_line[RX_LINE_SIZE];
static int rx_len = 0;

static void ResetFrameData() {
    red_count = 0;
    blue_count = 0;
    can_count = 0;
}

static void StoreRed(int index, float angle, int area) {
    if (red_count >= MAX_OBJECTS) return;
    red_objs[red_count++] = {index, angle, area, 'N'};
}

static void StoreBlue(int index, float angle, int area) {
    if (blue_count >= MAX_OBJECTS) return;
    blue_objs[blue_count++] = {index, angle, area, 'N'};
}

static void StoreCan(int index, float angle, char orient, int area) {
    if (can_count >= MAX_OBJECTS) return;
    can_objs[can_count++] = {index, angle, area, orient};
}

static void PrintFrameData() {
    printf("---- frame %lu ----\n", (unsigned long)frame_id);

    for (int i = 0; i < red_count; i++) {
        printf("RED  idx=%d angle=%.1f area=%d\n",
               red_objs[i].index, red_objs[i].angle, red_objs[i].area);
    }

    for (int i = 0; i < blue_count; i++) {
        printf("BLUE idx=%d angle=%.1f area=%d\n",
               blue_objs[i].index, blue_objs[i].angle, blue_objs[i].area);
    }

    for (int i = 0; i < can_count; i++) {
        printf("CAN  idx=%d angle=%.1f orient=%c area=%d\n",
               can_objs[i].index, can_objs[i].angle, can_objs[i].orient, can_objs[i].area);
    }
}

static void HandleLine(const char* line) {
    if (line == nullptr || line[0] == '\0') return;

    // フレーム開始
    if (strncmp(line, "F,", 2) == 0) {
        unsigned long fid = 0;
        if (sscanf(line, "F,%lu", &fid) == 1) {
            frame_id = (uint32_t)fid;
        }
        ResetFrameData();
        return;
    }

    // フレーム終了
    if (strcmp(line, "E") == 0) {
        PrintFrameData();

        // 必要ならここで display.hpp 側へ渡す
        // 例:
        // DisplaySetText(...);
        // DisplayUpdate();
        return;
    }

    // 赤: R,index,angle,area
    if (line[0] == 'R' && line[1] == ',') {
        int idx = 0;
        float angle = 0.0f;
        int area = 0;
        if (sscanf(line, "R,%d,%f,%d", &idx, &angle, &area) == 3) {
            StoreRed(idx, angle, area);
            printf("recv RED  idx=%d angle=%.1f area=%d\n", idx, angle, area);
        }
        return;
    }

    // 青: B,index,angle,area
    if (line[0] == 'B' && line[1] == ',') {
        int idx = 0;
        float angle = 0.0f;
        int area = 0;
        if (sscanf(line, "B,%d,%f,%d", &idx, &angle, &area) == 3) {
            StoreBlue(idx, angle, area);
            printf("recv BLUE idx=%d angle=%.1f area=%d\n", idx, angle, area);
        }
        return;
    }

    // 缶: C,index,angle,orient,area
    if (line[0] == 'C' && line[1] == ',') {
        int idx = 0;
        float angle = 0.0f;
        char orient_str[2] = {'N', '\0'};
        int area = 0;

        if (sscanf(line, "C,%d,%f,%1s,%d", &idx, &angle, orient_str, &area) == 4) {
            StoreCan(idx, angle, orient_str[0], area);
            printf("recv CAN  idx=%d angle=%.1f orient=%c area=%d\n",
                   idx, angle, orient_str[0], area);
        }
        return;
    }
}

// カメラの初期化
void CameraSetup() {
    uart_init(camera_uart, CAMERA_BAUD);
    gpio_set_function(camera_TX_pin, GPIO_FUNC_UART);  // TXピン
    gpio_set_function(camera_RX_pin, GPIO_FUNC_UART);  // RXピン
    uart_set_format(camera_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(camera_uart, true);

    rx_len = 0;
    ResetFrameData();
}

// カメラからの情報を受け取る(UART)
// その情報を使える形に加工する
// ディスプレイで表示する or シリアルモニタで表示するための処理
void UseCamera() {
    while (uart_is_readable(camera_uart)) {
        char ch = (char)uart_getc(camera_uart);

        if (ch == '\r' || ch == '\n') {
            if (rx_len > 0) {
                rx_line[rx_len] = '\0';
                HandleLine(rx_line);
                rx_len = 0;
            }
        } else {
            if (rx_len < RX_LINE_SIZE - 1) {
                rx_line[rx_len++] = ch;
            } else {
                // 長すぎる場合は破棄
                rx_len = 0;
            }
        }
    }
}