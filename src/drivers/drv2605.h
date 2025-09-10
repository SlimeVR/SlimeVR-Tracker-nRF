#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DRV2605_ADDR        (0x5A)
#define DRV2605_WAVEFORM_SLOTS  8

#define DRV2605_REG_STATUS         0x00
#define DRV2605_REG_MODE           0x01
#define DRV2605_REG_RTP_INPUT      0x02
#define DRV2605_REG_LIBRARY        0x03
#define DRV2605_REG_WAVESEQ1       0x04  // slots 0-7 (0x04-0x0B)
#define DRV2605_REG_GO             0x0C
#define DRV2605_REG_FEEDBACK       0x1A
#define DRV2605_REG_CONTROL1       0x1B
#define DRV2605_REG_CONTROL2       0x1C
#define DRV2605_REG_AUTOCAL_COMP   0x16
#define DRV2605_REG_AUTOCAL_BEMF   0x17
#define DRV2605_REG_AUTOCAL_BACKEMF 0x18
#define DRV2605_REG_AUTOCAL_IDISS  0x19

bool drv2605_init(void);
bool drv2605_play_effect(uint8_t effect_id);
void drv2605_stop(void);

// Diagnostics
bool drv2605_read_register(uint8_t reg, uint8_t *val);
void drv2605_report_status(void);

// Custom vibration sequence
bool drv2605_play_sequence(const uint8_t *effect_ids, uint8_t len);
