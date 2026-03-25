#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    sleep_ms(3000);  // ← これが超重要（USB接続待ち）

    while (true) {
        printf("hello\n");
        sleep_ms(1000);
    }
}