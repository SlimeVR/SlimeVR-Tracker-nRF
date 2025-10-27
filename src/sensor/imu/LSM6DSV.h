#ifndef LSM6DSV_h
#define LSM6DSV_h

#include "sensor/sensor.h"

// https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
#define LSM6DSV_IF_CFG                     0x03

#define LSM6DSV_FIFO_CTRL3                 0x09
#define LSM6DSV_FIFO_CTRL4                 0x0A
#define LSM6DSV_COUNTER_BDR_REG1           0x0B

#define LSM6DSV_INT1_CTRL                  0x0D

#define LSM6DSV_CTRL1                      0x10
#define LSM6DSV_CTRL2                      0x11
#define LSM6DSV_CTRL3                      0x12
#define LSM6DSV_CTRL6                      0x15
#define LSM6DSV_CTRL8                      0x17
#define LSM6DSV_CTRL9                      0x18

#define LSM6DSV_FIFO_STATUS1               0x1B
#define LSM6DSV_STATUS_REG                 0x1E

#define LSM6DSV_OUT_TEMP_L                 0x20
#define LSM6DSV_OUTX_L_G                   0x22
#define LSM6DSV_OUTX_L_A                   0x28
#define LSM6DSV_OUTX_H_A                   0x29

#define LSM6DSV_STATUS_MASTER_MAINPAGE     0x48
#define LSM6DSV_INTERNAL_FREQ_FINE         0x4F

#define LSM6DSV_FUNCTIONS_ENABLE           0x50
#define LSM6DSV_TAP_CFG0                   0x56
#define LSM6DSV_WAKE_UP_THS                0x5B
#define LSM6DSV_MD1_CFG                    0x5E

#define LSM6DSV_FIFO_DATA_OUT_TAG          0x78

// Sensor Hub
#define LSM6DSV_FUNC_CFG_ACCESS            0x01
#define LSM6DSV_SENSOR_HUB_1               0x02
#define LSM6DSV_MASTER_CONFIG              0x14
#define LSM6DSV_SLV0_ADD                   0x15
#define LSM6DSV_SLV0_SUBADD                0x16
#define LSM6DSV_SLV0_CONFIG                0x17
#define LSM6DSV_DATAWRITE_SLV0             0x21
#define LSM6DSV_STATUS_MASTER              0x22

// Same for XL and G
#define ODR_OFF     0x00
#define ODR_1_875Hz 0x01
#define ODR_7_5Hz   0x02
#define ODR_15Hz    0x03
#define ODR_30Hz    0x04
#define ODR_60Hz    0x05
#define ODR_120Hz   0x06
#define ODR_240Hz   0x07
#define ODR_480Hz   0x08
#define ODR_960Hz   0x09
#define ODR_1_92kHz 0x0A
#define ODR_3_84kHz 0x0B
#define ODR_7_68kHz 0x0C

#define OP_MODE_XL_HP     0x00 // High Performance
#define OP_MODE_XL_HA     0x01 // High Accuracy
#define OP_MODE_XL_ODR_T  0x03 // ODR-Triggered
#define OP_MODE_XL_LP1    0x04 // Low Power mode 1 (2 mean)
#define OP_MODE_XL_LP2    0x05 // Low Power mode 2 (4 mean)
#define OP_MODE_XL_LP3    0x06 // Low Power mode 3 (8 mean)
#define OP_MODE_XL_NORMAL 0x07

#define OP_MODE_G_HP    0x00 // High Performance
#define OP_MODE_G_HA    0x01 // High Accuracy
#define OP_MODE_G_SLEEP 0x04
#define OP_MODE_G_LP    0x05 // Low Power

#define FS_G_125DPS  0x00
#define FS_G_250DPS  0x01
#define FS_G_500DPS  0x02
#define FS_G_1000DPS 0x03
#define FS_G_2000DPS 0x04
#define FS_G_4000DPS 0x0C

#define FS_XL_2G  0x00
#define FS_XL_4G  0x01
#define FS_XL_8G  0x02
#define FS_XL_16G 0x03

// TODO: shared with common LSM
extern float accel_sensitivity; // Default 16G (FS = ±16 g: 0.488 mg/LSB)
extern float gyro_sensitivity; // Default 2000dps (FS = ±2000 dps: 70 mdps/LSB)

extern uint8_t last_accel_mode;
extern uint8_t last_gyro_mode;
extern uint8_t last_accel_odr;
extern uint8_t last_gyro_odr;

int lsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void lsm_shutdown(void);

void lsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range);
int lsm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t lsm_fifo_read(uint8_t *data, uint16_t len);
int lsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void lsm_accel_read(float a[3]);
void lsm_gyro_read(float g[3]);
float lsm_temp_read(void);

uint8_t lsm_setup_DRDY(uint16_t threshold);
uint8_t lsm_setup_WOM(void);

int lsm_ext_setup(void);
int lsm_ext_passthrough(bool passthrough);

int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes);
int lsm_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

extern const sensor_imu_t sensor_imu_lsm6dsv;
extern const sensor_ext_ssi_t sensor_ext_lsm6dsv;

#endif
