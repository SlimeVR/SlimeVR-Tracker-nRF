#ifndef BMI270_h
#define BMI270_h

#include "sensor/sensor.h"

#define BMI270_INTERNAL_STATUS 0x21

#define BMI270_DATA_8  0x0C // accel
#define BMI270_DATA_14 0x12 // gyro
#define BMI270_TEMPERATURE_0 0x22

#define BMI270_FIFO_LENGTH_0 0x24
#define BMI270_FIFO_DATA 0x26

#define BMI270_FEAT_PAGE 0x2F
// page 0
#define BMI270_GYR_GAIN_STATUS 0x38
#define BMI270_GYR_CAS 0x3C
// page 1
#define BMI270_G_TRIG_1 0x32
#define BMI270_ANYMO_1 0x3C
#define BMI270_ANYMO_2 0x3E

#define BMI270_ACC_CONF  0x40
#define BMI270_ACC_RANGE 0x41
#define BMI270_GYR_CONF  0x42
#define BMI270_GYR_RANGE 0x43

#define BMI270_FIFO_WTM_0 0x46
#define BMI270_FIFO_CONFIG_0 0x48
#define BMI270_FIFO_CONFIG_1 0x49

#define BMI270_INT1_IO_CTRL 0x53
#define BMI270_INT1_MAP_FEAT 0x56
#define BMI270_INT_MAP_DATA 0x58

#define BMI270_INIT_CTRL 0x59
#define BMI270_INIT_ADDR_0 0x5B
#define BMI270_INIT_ADDR_1 0x5C
#define BMI270_INIT_DATA 0x5E

#define BMI270_GYR_CRT_CONF 0x69
#define BMI270_OFFSET_6  0x77
#define BMI270_GYR_USR_GAIN 0x78

#define BMI270_PWR_CONF  0x7C
#define BMI270_PWR_CTRL  0x7D
#define BMI270_CMD       0x7E

// accel only
#define ODR_0p78 0x01
#define ODR_1p5  0x02
#define ODR_3p1  0x03
#define ODR_6p25 0x04
#define ODR_12p5 0x05
// accel and gyro
#define ODR_25   0x06
#define ODR_50   0x07
#define ODR_100  0x08
#define ODR_200  0x09
#define ODR_400  0x0a
#define ODR_800  0x0b
#define ODR_1k6  0x0c
// gyro only
#define ODR_3k2  0x0d

#define RANGE_2G  0x00
#define RANGE_4G  0x01
#define RANGE_8G  0x02
#define RANGE_16G 0x03

#define RANGE_2000 0x00
#define RANGE_1000 0x01
#define RANGE_500  0x02
#define RANGE_250  0x03
#define RANGE_125  0x04

int bmi270_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void bmi270_shutdown(void);

void bmi270_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range);
int bmi270_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t bmi270_fifo_read(uint8_t *data, uint16_t len);
int bmi270_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void bmi270_accel_read(float a[3]);
void bmi270_gyro_read(float g[3]);
float bmi270_temp_read(void);

uint8_t bmi270_setup_DRDY(uint16_t threshold);
uint8_t bmi270_setup_WOM(void);

int bmi270_crt(uint8_t *data);
void bmi270_gain_apply(uint8_t *data);

extern const sensor_imu_t sensor_imu_bmi270;

#endif
