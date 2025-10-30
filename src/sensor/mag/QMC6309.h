#ifndef QMC6309_h
#define QMC6309_h

#include "sensor/sensor.h"

int qmc6309_init(float time, float *actual_time);
void qmc6309_shutdown(void);

int qmc6309_update_odr(float time, float *actual_time);

void qmc6309_mag_oneshot(void);
void qmc6309_mag_read(float m[3]);

void qmc6309_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc6309;

#endif
