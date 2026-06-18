#ifndef QMC5883_h
#define QMC5883_h

#include "sensor/sensor.h"

#define QMC588P_DEVADDR          0x2C
#define QMC5883P_CHIP_ID_REG     0x00
#define QMC5883P_OUTX_L_REG      0x01
#define QMC5883P_OUTX_M_REG      0x02
#define QMC5883P_OUTY_L_REG      0x03
#define QMC5883P_OUTY_M_REG      0x04
#define QMC5883P_OUTZ_L_REG      0x05
#define QMC5883P_OUTZ_M_REG      0x06
#define QMC5883P_STATUS_REG      0x09
#define QMC5883P_CTRL1_REG       0x0A
#define QMC5883P_CTRL2_REG       0x0B
#define QMC5883P_AXIS_SIGN_REG   0x29

// Status bits
#define QMC5883P_STAT_DRDY       0x01
#define QMC5883P_STAT_OVFL       0x02

// Control1 bits: OSR2<1:0> OSR1<1:0> ODR<1:0> MODE<1:0>
#define QMC5883P_MODE_SUSPEND    0x00
#define QMC5883P_MODE_NORMAL     0x01
#define QMC5883P_MODE_SINGLE     0x02
#define QMC5883P_MODE_CONT       0x03

// Control2 bits: SOFT_RST SELF_TEST -- RNG<1:0> SET/RESET<1:0>
#define QMC5883P_SOFT_RST        0x80
#define QMC5883P_SELF_TEST       0x40
#define QMC5883P_RNG_2G          (0b11 << 4)
#define QMC5883P_RNG_8G          (0b10 << 4)
#define QMC5883P_RNG_12G         (0b01 << 4)
#define QMC5883P_RNG_30G         (0b00 << 4)
#define QMC5883P_SET_RESET_ON    0x00   // per Table 18:contentReference[oaicite:1]{index=1}
#define QMC5883P_SET_ONLY        0x10

int qmc5883p_init(float time, float *actual_time);
void qmc5883p_shutdown(void);

int qmc5883p_update_odr(float time, float *actual_time);

void qmc5883p_mag_oneshot(void);

void qmc5883p_mag_read(float m[3]);

void qmc5883p_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc5883p;

#endif
