#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

#include "interface.h"
#include "sensor_position.h"

enum sensor_loop_state_e
{
    SENSOR_LOOP_DISCOVERING,
    // DISCOVERING_SPI,
    // DISCOVERING_I2C,
    // DISCOVERING_EXT,
    SENSOR_LOOP_INITIALIZING_IMU,
    // INITIALIZING_IMU_SPI,
    // INITIALIZING_IMU_I2C,
    // INITIALIZING_IMU_EXT,
    SENSOR_LOOP_INITIALIZING_MAG,
    // INITIALIZING_MAG_SPI,
    // INITIALIZING_MAG_I2C,
    // INITIALIZING_MAG_EXT,
    SENSOR_LOOP_READING_IMU,
    // READING_IMU_SPI,
    // READING_IMU_I2C,
    // READING_IMU_EXT,
    SENSOR_LOOP_READING_MAG,
    // READING_MAG_SPI,
    // READING_MAG_I2C,
    // READING_MAG_EXT
};

typedef enum
{
    SENSOR_EXT_MODE_OFF = 0,             // no external sensor
    SENSOR_EXT_MODE_I2C_PASSTHROUGH = 1, // shorting external i2c bus to main i2c bus, mcu controls everything
    SENSOR_EXT_MODE_I2CM_PROXY = 2,      // separate buses, imu is as as a proxy / protocol to i2c converter
    SENSOR_EXT_MODE_I2CM_AUTONOMOUS = 4, // separate buses, mag is autonomically driven by imu, data lands into fifo
} sensor_ext_mode_t;

typedef enum
{
    DATA_VALID_ACCEL = 1,
    DATA_VALID_GYRO = 2,
    DATA_VALID_MAG = 4,
    DATA_OUTSIDE_FIFO = 8,
    DATA_INVALID = 16,
} sensor_data_attrs_t;

enum sensor_sensor_timeout
{
    SENSOR_SENSOR_TIMEOUT_IMU,
    SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED,
    SENSOR_SENSOR_TIMEOUT_ACTIVITY,
    SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED,
};

enum sensor_sensor_mode
{
    //	SENSOR_SENSOR_MODE_OFF,
    SENSOR_SENSOR_MODE_LOW_NOISE,
    SENSOR_SENSOR_MODE_LOW_POWER,
    SENSOR_SENSOR_MODE_LOW_POWER_2
};

typedef struct sensor_fusion
{
    void (*init)(float, float, float); // gyro_time, accel_time, mag_time
    void (*load)(const void *);
    void (*save)(void *);

    void (*update_gyro)(float *, float);  // deg/s
    void (*update_accel)(float *, float); // g
    void (*update_mag)(float *, float);   // any unit (usually gauss)
    void (*update)(float *, float *, float *, float);

    void (*get_gyro_bias)(float *);
    void (*set_gyro_bias)(float *);

    void (*update_gyro_sanity)(float *, float *);
    int (*get_gyro_sanity)(void);

    void (*get_lin_a)(float *);
    void (*get_quat)(float *);
} sensor_fusion_t;

typedef struct sensor_mag
{
    int (*init)(float, float *); // return update time, return 0 if success, 1 if general error
    void (*shutdown)(void);

    int (*update_odr)(float, float *); // change update time, return real one, return 0 if success, 1 if odr is same, -1 if general error
    float (*get_odr)(void);            // return real update time

    void (*mag_oneshot)(void);    // trigger oneshot if exists
    void (*mag_read)(float[3]);   // any unit (usually gauss)
    float (*temp_read)(float[3]); // deg C

    void (*mag_process)(uint8_t *, float[3]); // use if magnetometer is present as an auxiliary sensor, from data read by IMU
    uint8_t ext_min_burst;                    // minimum supported burst length for external interface
    uint8_t ext_burst;                        // default supported burst length
    uint8_t ext_dummy_bytes;                  // required dummy bytes to skip during every read
    uint8_t ext_burst_reg;                    // first register address to read in burst
} sensor_mag_t;

typedef struct sensor_imu
{
    int (*init)(float, float, float, float *, float *); // first float is clock_rate, nonzero means use CLKIN, return update time, return 0 if success, -1 if general error
    void (*shutdown)(void);

    void (*update_fs)(float, float, float *, float *); // return actual range
    int (*update_odr)(float, float, float *, float *); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

    uint16_t (*data_read)(uint8_t *, uint16_t);
    sensor_data_attrs_t (*data_process)(uint16_t, uint8_t *, float[3], float[3], float[3]); // g, deg/s
    void (*accel_read)(float[3]);                                                           // g
    void (*gyro_read)(float[3]);                                                            // deg/s
    int (*temp_read)(float *);                                                              // deg C, return 0 if success, -1 if error

    uint8_t (*setup_DRDY)(uint16_t);
    uint8_t (*setup_WOM)(void);

    int (*ext_setup)(sensor_ext_mode_t, const sensor_mag_t *mag, uint8_t mag_addr); // mag used for autonomous mode
} sensor_imu_t;

typedef struct
{

    sensor_imu_t *sensor_imu;
    sensor_mag_t *sensor_mag;
    sensor_fusion_t *sensor_fusion;

    struct spi_dt_spec *sensor_imu_dev;

    int sensor_imu_id;
    int sensor_mag_id;

    sensor_position_e sensor_position;
    char *sensor_name;

    uint8_t sensor_mag_dev_reg;

    float q[4];      // vector to hold quaternion
    float last_q[4]; // vector to hold quaternion

    float q3[4]; // correction quaternion

    float last_lin_a[3]; // vector to hold last linear accelerometer

    float last_m[3];
    int64_t last_mag_time;
    int64_t mag_interval;

    float temp; // sensor temperature
    int64_t last_temp_time;

    int64_t last_suspend_attempt_time;
    int64_t last_data_time;

    int64_t time_begin;
    int64_t last_status_time;
    int64_t max_loop_time;

    float accel_actual_time;
    float gyro_actual_time;
    float mag_actual_time;

    float sensor_actual_time;
    int16_t sensor_fifo_threshold;
    int64_t sensor_data_time; // ticks
    int64_t sensor_update_time_ms;

    int packet_errors;

    bool main_ok;
    bool mag_enabled; // TODO: toggle from server
    bool mag_available;
    bool mag_manual;
    bool sensor_fusion_init;
    bool sensor_init;

    int fusion_id;

    enum sensor_sensor_mode sensor_mode;
    enum sensor_sensor_mode last_sensor_mode;
    enum sensor_sensor_timeout sensor_timeout;
    sensor_position_e sensor_positon;

} sensor_dev_t;

enum sensor_loop_state_e sensor_loop_get_state(void);
void sensor_loop_set_state(enum sensor_loop_state_e state);

const char *sensor_get_sensor_imu_name(sensor_dev_t *sensor_dev);
const char *sensor_get_sensor_mag_name(sensor_dev_t *sensor_dev);
const char *sensor_get_sensor_fusion_name(sensor_dev_t *sensor_dev);

int sensor_get_sensor_temperature(float *, sensor_dev_t *sensor_dev);

int sensor_request_scan(bool force);

void sensor_scan_read(void);
void sensor_scan_write(void);
void sensor_scan_clear(void);

void sensor_retained_read(sensor_dev_t *sensor_dev);
void sensor_retained_write(sensor_dev_t *sensor_dev);

void sensor_shutdown(void);
uint8_t sensor_setup_WOM(void);

void sensor_fusion_invalidate_all(void);

void sensor_fusion_invalidate(sensor_dev_t *sensor_dev);

void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_resume(void);
void main_imu_wakeup(void);
void main_imu_restart(sensor_dev_t *sensor_dev);

#endif