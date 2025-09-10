#include "drv2605.h"
#include "uart_hal.h"
#include "i2c.h" // SlimeVR's I2C abstraction

static inline bool drv2605_write(uint8_t reg, uint8_t val) {
    return i2c_write_register(DRV2605_ADDR, reg, val);
}

bool drv2605_read_register(uint8_t reg, uint8_t *val) {
    return i2c_read_register(DRV2605_ADDR, reg, val);
}

bool drv2605_init(void) {
    if (!drv2605_write(DRV2605_REG_MODE, 0x00)) return false;
    if (!drv2605_write(DRV2605_REG_FEEDBACK, 0x7F)) return false;
    if (!drv2605_write(DRV2605_REG_LIBRARY, 1)) return false;
    if (!drv2605_write(DRV2605_REG_CONTROL1, 0x93)) return false;
    if (!drv2605_write(DRV2605_REG_CONTROL2, 0xF5)) return false;
    return true;
}

bool drv2605_play_effect(uint8_t effect_id) {
    if (!drv2605_write(DRV2605_REG_WAVESEQ1, effect_id)) return false;
    if (!drv2605_write(DRV2605_REG_WAVESEQ1 + 1, 0)) return false;
    if (!drv2605_write(DRV2605_REG_GO, 1)) return false;
    return true;
}

bool drv2605_play_sequence(const uint8_t *effect_ids, uint8_t len) {
    if (len > DRV2605_WAVEFORM_SLOTS) len = DRV2605_WAVEFORM_SLOTS;
    for (uint8_t i = 0; i < len; i++) {
        if (!drv2605_write(DRV2605_REG_WAVESEQ1 + i, effect_ids[i])) return false;
    }
    if (!drv2605_write(DRV2605_REG_WAVESEQ1 + len, 0)) return false;
    if (!drv2605_write(DRV2605_REG_GO, 1)) return false;
    return true;
}

void drv2605_stop(void) {
    drv2605_write(DRV2605_REG_GO, 0);
}

void drv2605_report_status(void) {
    const struct {
        const char *name;
        uint8_t reg;
    } regs[] = {
        {"STATUS",      DRV2605_REG_STATUS},
        {"MODE",        DRV2605_REG_MODE},
        {"RTP_INPUT",   DRV2605_REG_RTP_INPUT},
        {"LIBRARY",     DRV2605_REG_LIBRARY},
        {"WAVESEQ1",    DRV2605_REG_WAVESEQ1},
        {"GO",          DRV2605_REG_GO},
        {"FEEDBACK",    DRV2605_REG_FEEDBACK},
        {"CONTROL1",    DRV2605_REG_CONTROL1},
        {"CONTROL2",    DRV2605_REG_CONTROL2},
        {"AUTOCAL_COMP", DRV2605_REG_AUTOCAL_COMP},
        {"AUTOCAL_BEMF", DRV2605_REG_AUTOCAL_BEMF},
        {"AUTOCAL_BACKEMF", DRV2605_REG_AUTOCAL_BACKEMF},
        {"AUTOCAL_IDISS", DRV2605_REG_AUTOCAL_IDISS}
    };
    uint8_t val;
    for (unsigned i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
        if (drv2605_read_register(regs[i].reg, &val))
            uart_hal_printf("%s: 0x%02X\r\n", regs[i].name, val);
        else
            uart_hal_printf("%s: ERROR\r\n", regs[i].name);
    }
}