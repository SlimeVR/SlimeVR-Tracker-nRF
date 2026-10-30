#ifndef SLIMENRF_SENSOR_READ
#define SLIMENRF_SENSOR_READ

#include "sensor.h"
#include "calibration.h"

void process_imu_packets(sensor_dev_t *sensor_dev, uint16_t *packets, uint8_t *raw_data, float *raw_m, int *g_count, int *a_count, float *a_sum, int *processed_packets, bool *mag_read);

void process_imu_orientation();

void process_magnetometer_calibration();

#endif