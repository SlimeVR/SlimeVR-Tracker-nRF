#include "drv2605.h"
#include "uart_hal.h"

int main(void) {
    uart_hal_init();

    drv2605_init();

    // Example: Play a single effect (ID 1 = Strong Click)
    drv2605_play_effect(1);

    // Example: Play custom sequence using DRV2605L effect table (max 8 slots)
    uint8_t seq[8] = {
        1,   // Strong Click – 100%
        4,   // Sharp Click – 100%
        7,   // Soft Bump – 100%
        10,  // Double Click – 100%
        47,  // Ramp Up
        87,  // Pulse
        93,  // Hum 2
        110  // Transition Click
    };
    drv2605_play_sequence(seq, 8);

    // Example: Report all registers to UART
    drv2605_report_status();

    while (1) {
        // Main loop code here
    }
}