#include "globals.h"
#include "system/system.h"
#include "util.h"
#include "connection/connection.h"
#include "calibration.h"

#include <math.h>
#include <hal/nrf_gpio.h>

#include "fusion/fusions.h"
#include "sensors.h"

#include "sensor.h"

LOG_MODULE_REGISTER(sensor_event, LOG_LEVEL_DBG);

#define SPI_OP SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, imu_count)
#define IMU_COUNT DT_PROP(ZEPHYR_USER_NODE, imu_count)
#define MULTIPLE_SENSORS true
// Black Magic
#define IMU_SPI_SPEC(i, _) SPI_DT_SPEC_GET(DT_NODELABEL(imu_spi_##i), SPI_OP, 0)
struct imu_spi_entry
{
    struct spi_dt_spec spec;
    const char *name;
    const sensor_position_e sensor_position;
};

#define IMU_SPI_ENTRY(i, _) {                                               \
    .spec = SPI_DT_SPEC_GET(DT_NODELABEL(imu_spi_##i), SPI_OP, 0),          \
    .name = DT_PROP(DT_NODELABEL(imu_spi_##i), label),                      \
    .sensor_position = DT_PROP(DT_NODELABEL(imu_spi_##i), sensor_position), \
}

static struct imu_spi_entry sensor_imu_spi_devs[] = {
    LISTIFY(IMU_COUNT, IMU_SPI_ENTRY, (, ))};
int imu_ids[IMU_COUNT];
static struct spi_dt_spec sensor_imu_spi_dev;
#else
#define IMU_COUNT 1
#endif

#if SENSOR_IMU_EXISTS
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif

#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS && !MULTIPLE_SENSORS
#error "No IMU node(s) exist"
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if SENSOR_MAG_SPI_EXISTS
#define SENSOR_MAG_SPI_NODE DT_NODELABEL(mag_spi)
static struct spi_dt_spec sensor_mag_spi_dev = SPI_DT_SPEC_GET(SENSOR_MAG_SPI_NODE, SPI_OP, 0);
#endif

#if SENSOR_DIRECT_MAG_EXISTS
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif

#if !SENSOR_MAG_EXISTS
#warning "Magnetometer does not exist"
#endif

static uint8_t sensor_mag_dev_reg = 0xFF;

#define ACQUISITION_START_MS 1000
#define STATUS_INTERVAL_MS 5000

static bool main_suspended;
static bool main_running;

static sensor_dev_t connected_sensors[IMU_COUNT];

static void sensor_thread(void);

static enum sensor_loop_state_e sensor_loop_state = SENSOR_LOOP_DISCOVERING;
static uint32_t sensor_loop_state_last_change = 0;

K_THREAD_DEFINE(sensor_thread_id, 8192, sensor_thread, true, NULL, NULL, SENSOR_THREAD_PRIORITY, K_FP_REGS, 0);

enum sensor_loop_state_e sensor_loop_get_state(void)
{
    return sensor_loop_state;
}

void sensor_loop_set_state(enum sensor_loop_state_e state)
{
    enum sensor_loop_state_e old_state = sensor_loop_state;
    sensor_loop_state_last_change = k_uptime_get_32();
    sensor_loop_state = state;
    LOG_INF("Sensor loop state change: %d -> %d", old_state, state);
}

const char *sensor_get_sensor_imu_name(sensor_dev_t *sensor_dev)
{
    if (sensor_dev->sensor_imu_id < 0)
        return "\033[38;5;196;1mNone\033[0m"; // color 196 (bright red), intense/bold
    return dev_imu_names[sensor_dev->sensor_imu_id];
}

const char *sensor_get_sensor_mag_name(sensor_dev_t *sensor_dev)
{
    if (sensor_dev->sensor_mag_id < 0)
        return "None";
    return dev_mag_names[sensor_dev->sensor_mag_id];
}

const char *sensor_get_sensor_fusion_name(sensor_dev_t *sensor_dev)
{
    if (sensor_dev->fusion_id < 0 || sensor_dev->fusion_id >= FUSION_COUNT)
        return "None";
    return fusion_names[sensor_dev->fusion_id];
}

static void set_update_time_ms(int time_ms, sensor_dev_t *sensor_dev)
{
    // TODO: maybe not get rid of it? it is now repurposed to also change FIFO threshold
    // TODO: return pin_config and replace call in sensor_init
#if IMU_INT_EXISTS
    float fifo_threshold = time_ms / 1000.0f / sensor_actual_time; // target loop rate
    sensor_fifo_threshold = fifo_threshold;
    LOG_DBG("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_fifo_threshold);
    sensor_imu->setup_DRDY(sensor_fifo_threshold); // do not need to reset pin config
#endif
    sensor_dev->sensor_update_time_ms = time_ms; // TODO: terrible naming
}

// Check the IMU gyroscope // TODO: gyro sanity not used
// TODO: timeouts and power management should be outside sensor! (e.g. sleeping/shutdown even if the imu completely errored out)
// all this really means is that this should be called in sensor loop while the sensor is in an error state
static void sensor_update_sensor_state(sensor_dev_t *sensor_dev)
{
    bool calibrating = get_status(SYS_STATUS_CALIBRATION_RUNNING);
    bool resting = sensor_dev->sensor_fusion->get_gyro_sanity() == 0 ? q_epsilon(sensor_dev->q, sensor_dev->last_q, 0.005) : q_epsilon(sensor_dev->q, sensor_dev->last_q, 0.05); // TODO: Probably okay to use the constantly updating last_q?
    if (!calibrating && resting)
    {
        int64_t last_data_delta = k_uptime_get() - sensor_dev->last_data_time;
        if (sensor_dev->sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER && last_data_delta > CONFIG_3_SETTINGS_READ(CONFIG_3_SENSOR_LP_TIMEOUT)) // No motion in lp timeout
        {
            LOG_INF("No motion from sensors in %dms", CONFIG_3_SETTINGS_READ(CONFIG_3_SENSOR_LP_TIMEOUT));
            sensor_dev->sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
        }
        int64_t imu_timeout = CLAMP(sensor_dev->last_data_time - sensor_dev->last_suspend_attempt_time, CONFIG_3_SETTINGS_READ(CONFIG_3_IMU_TIMEOUT_RAMP_MIN), CONFIG_3_SETTINGS_READ(CONFIG_3_IMU_TIMEOUT_RAMP_MAX)); // Ramp timeout from last_data_time
        if (CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_LOW_POWER_2) && sensor_dev->sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER_2 && last_data_delta > imu_timeout)                                                      // No motion in ramp time
            sensor_dev->sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER_2;
        if (CONFIG_1_SETTINGS_READ(CONFIG_1_USE_ACTIVE_TIMEOUT))
        {
            if (sensor_dev->sensor_timeout < SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_3_SETTINGS_READ(CONFIG_3_ACTIVE_TIMEOUT_THRESHOLD)) // higher priority than IMU timeout
            {
                LOG_INF("Switching to activity timeout");
                sensor_dev->sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
            }
            if (sensor_dev->sensor_timeout == SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_3_SETTINGS_READ(CONFIG_3_ACTIVE_TIMEOUT_DELAY))
            {
                LOG_INF("No motion from sensors in %dm", CONFIG_3_SETTINGS_READ(CONFIG_3_ACTIVE_TIMEOUT_DELAY) / 60000);
                // Queue power state request, it is possible for the request to be overridden so the thread may continue unaware
                if (CONFIG_2_SETTINGS_READ(CONFIG_2_ACTIVE_TIMEOUT_MODE) == 0 && CONFIG_0_SETTINGS_READ(CONFIG_0_USE_IMU_WAKE_UP))
                    sys_request_WOM(true, false);
                // Queue power state request, thread will be suspended when entering system_off
                if (CONFIG_2_SETTINGS_READ(CONFIG_2_ACTIVE_TIMEOUT_MODE) == 1 && CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN))
                    sys_request_system_off(false);
                sensor_dev->sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED; // only try to suspend once
            }
        }
        if (CONFIG_1_SETTINGS_READ(CONFIG_1_USE_IMU_TIMEOUT) && CONFIG_0_SETTINGS_READ(CONFIG_0_USE_IMU_WAKE_UP) && sensor_dev->sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU && last_data_delta > imu_timeout) // No motion in ramp time
        {
            LOG_INF("No motion from sensors in %llds", imu_timeout / 1000);
            // Queue power state request
            sys_request_WOM(false, false);
            sensor_dev->sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED; // only try to suspend once
        }
        // Update timeout estimate
        switch (sensor_dev->sensor_timeout)
        {
        case SENSOR_SENSOR_TIMEOUT_ACTIVITY:
            connection_update_sensor_timeout_time(CONFIG_3_SETTINGS_READ(CONFIG_3_ACTIVE_TIMEOUT_DELAY) - last_data_delta);
            break;
        case SENSOR_SENSOR_TIMEOUT_IMU:
            if (CONFIG_1_SETTINGS_READ(CONFIG_1_USE_IMU_TIMEOUT) && CONFIG_0_SETTINGS_READ(CONFIG_0_USE_IMU_WAKE_UP))
            {
                connection_update_sensor_timeout_time(imu_timeout - last_data_delta);
                break;
            }
        default:
            connection_update_sensor_timeout_time(INT64_MAX);
        }
    }
    else
    {
        if (sensor_dev->sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER_2 || sensor_dev->sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED)
            sensor_dev->last_suspend_attempt_time = k_uptime_get();
        sensor_dev->last_data_time = k_uptime_get();
        if (sensor_dev->sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) // Resetting IMU timeout
            sensor_dev->sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
        sensor_dev->sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
        connection_update_sensor_timeout_time(INT64_MAX);
    }
}

/**
 * @param ptr last read sensor temperature
 * @return int - 0 if success, -1 if there is IMU errors fetching the temperature or it's too early,
 * -2 if there is no IMU connected
 */
int sensor_get_sensor_temperature(float *ptr, sensor_dev_t *sensor_dev)
{
    if (sensor_dev->sensor_imu == &sensor_imu_none || (k_uptime_get() - sensor_dev->last_temp_time > 1000))
    {
        *ptr = 25.0f; // fallback
        if (get_status(SYS_STATUS_SENSOR_ERROR))
            return -2; // no imu!
        else
            return -1; // imu probably not scanned yet or temp not read yet or last valid temp is old
    }
    *ptr = sensor_dev->temp;
    return 0;
}

void wait_for_threads(void)
{
    while (main_running)
        k_usleep(1);
}

int sensor_request_scan(bool force)
{
    sensor_loop_set_state(SENSOR_LOOP_DISCOVERING);
    return 0;
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
    retained->imu_addr = sensor_imu_dev.addr;
    retained->mag_addr = sensor_mag_dev.addr;
    retained->imu_reg = sensor_imu_dev_reg;
    retained->mag_reg = sensor_mag_dev_reg;
    retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
    retained->imu_addr = 0x00;
    retained->mag_addr = 0x00;
    retained->imu_reg = 0xFF;
    retained->mag_reg = 0xFF;
    retained_update();
}

void sensor_retained_read(sensor_dev_t *sensor_dev) // TODO: move some of this to sys? or move to calibration?
{
    if (CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_6_SIDE_CALIBRATION))
    {
        LOG_INF("Accelerometer matrix:");
        for (int i = 0; i < 3; i++)
            LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
    }
    else
    {
        LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
    }
    LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
    if (sensor_dev->mag_available && sensor_dev->mag_enabled)
    {
        //		LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
        LOG_INF("Magnetometer matrix:");
        for (int i = 0; i < 3; i++)
            LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
    }
    if (retained->fusion_id)
        LOG_INF("Fusion data recovered");
}

void sensor_retained_write(sensor_dev_t *sensor_dev) // TODO: move to sys?
{
    if (!sensor_dev->sensor_fusion_init)
        return;
    //	memcpy(retained->magBias, sensor_calibration_get_magBias(), sizeof(retained->magBias));
    sensor_dev->sensor_fusion->save(retained->fusion_data);
    retained->fusion_id = sensor_dev->fusion_id;
    retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
    int err = sensor_request_scan(false); // try initialization if possible
    for (int i = 0; i < IMU_COUNT; i++)
    {
        if (connected_sensors[i].mag_available || !err)
        {
            sys_interface_resume();
            if (connected_sensors[i].mag_available) // try to shutdown magnetometer first (in case of passthrough)
                connected_sensors[i].sensor_mag->shutdown();
            if (!err)
                connected_sensors[i].sensor_imu->shutdown();
            sys_interface_suspend();
        }
        else
        {
            LOG_ERR("Failed to shutdown sensors");
        }
    }
}

void main_imu_suspend(void) // TODO: add timeout
{
    main_suspended = true;
    if (!main_running) // don't suspend if already stopped (TODO: may be called from sensor thread)
        return;
    while (sensor_loop_get_state() == SENSOR_LOOP_DISCOVERING)
        k_usleep(1);     // try not to interrupt scanning
    while (main_running) // TODO: change to detect if i2c is busy
        k_usleep(1);     // try not to interrupt anything actually
    k_thread_suspend(&sensor_thread_id);
    LOG_INF("Suspended sensor thread");
}

void main_imu_resume(void)
{
    if (!main_suspended) // not suspended
        return;
    k_thread_resume(&sensor_thread_id);
    LOG_INF("Resumed sensor thread");
}

void main_imu_wakeup(void)
{
    if (!main_suspended) // don't wake up if pending suspension
        k_wakeup(&sensor_thread_id);
}

void main_imu_restart(sensor_dev_t *sensor_dev)
{
    if (sensor_dev->main_ok) // only restart fusion if initialized
        sensor_dev->sensor_fusion->init(sensor_dev->gyro_actual_time, sensor_dev->accel_actual_time, sensor_dev->mag_actual_time);
}

void sensor_fusion_invalidate_all(void)
{
    for (int i = 0; i < IMU_COUNT; i++)
    {
        sensor_fusion_invalidate(&connected_sensors[i]);
    }
}
void sensor_fusion_invalidate(sensor_dev_t *sensor_dev)
{
    main_imu_restart(sensor_dev); // reinitialize fusion
    if (sensor_dev->sensor_fusion_init)
    { // clear fusion gyro offset
        float g_off[3] = {0};
        sensor_dev->sensor_fusion->set_gyro_bias(g_off);
        sensor_retained_write(sensor_dev);
    }
    else
    {                            // TODO: always clearing the fusion?
        retained->fusion_id = 0; // Invalidate retained fusion data
        retained_update();
    }
}

void initialize_sensor_defaults(sensor_dev_t *sensor_dev)
{
    sensor_dev->sensor_imu = &sensor_imu_none;
    sensor_dev->sensor_mag = &sensor_mag_none;
    sensor_dev->sensor_fusion = &sensor_fusion_none;

    sensor_dev->sensor_imu_id = -1;
    sensor_dev->sensor_mag_id = -1;
    sensor_dev->fusion_id = 0;

    sensor_dev->sensor_position = -1;
    sensor_dev->sensor_name = "";

    memcpy(sensor_dev->q, (float[4]){1.0f, 0.0f, 0.0f, 0.0f}, sizeof(sensor_dev->q));
    memcpy(sensor_dev->last_q, (float[4]){1.0f, 0.0f, 0.0f, 0.0f}, sizeof(sensor_dev->last_q));

    memcpy(sensor_dev->q3, (float[4]){SENSOR_QUATERNION_CORRECTION}, sizeof(sensor_dev->q3));
    memcpy(sensor_dev->last_lin_a, (float[3]){0}, sizeof(sensor_dev->last_lin_a));
    memcpy(sensor_dev->last_m, (float[3]){0}, sizeof(sensor_dev->last_m));

    sensor_dev->last_mag_time = -1000;
    sensor_dev->mag_interval = 0;

    sensor_dev->last_temp_time = -1000;

    sensor_dev->last_suspend_attempt_time = 0;
}

int sensor_init(sensor_dev_t *sensor_dev)
{
    int err;
    LOG_INF("Sensor init, shutdown first");
    // TODO : Do not reset sensor if we just WOM'ed
    // TODO: on any errors set main_ok false and skip (make functions return nonzero)
    if (sensor_dev->mag_available) // shutdown magnetometer first (in case of passthrough)
        sensor_dev->sensor_mag->shutdown();
    sensor_dev->sensor_imu->shutdown();

    float clock_actual_rate = 0;
    if (CONFIG_1_SETTINGS_READ(CONFIG_1_USE_SENSOR_CLOCK))
        set_sensor_clock(true, 32768, &clock_actual_rate); // enable the clock source for IMU if present
    if (clock_actual_rate != 0)
        LOG_INF("Sensor clock rate: %.2fHz", (double)clock_actual_rate);

    // set FS/range
    float accel_range = CONFIG_2_SETTINGS_READ(CONFIG_2_SENSOR_ACCEL_FS);
    float gyro_range = CONFIG_2_SETTINGS_READ(CONFIG_2_SENSOR_GYRO_FS);
    float accel_actual_range, gyro_actual_range;
    sensor_dev->sensor_imu->update_fs(accel_range, gyro_range, &accel_actual_range, &gyro_actual_range);
    LOG_INF("Accelerometer range: %.2fg", (double)accel_actual_range);
    LOG_INF("Gyroscope range: %.2fdps", (double)gyro_actual_range);

    // get mag enable from config
    sensor_dev->mag_enabled = CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_MAG);

    // setup sensor, set ODR
    float accel_initial_time = 1.0 / CONFIG_2_SETTINGS_READ(CONFIG_2_SENSOR_ACCEL_ODR);
    float gyro_initial_time = 1.0 / CONFIG_2_SETTINGS_READ(CONFIG_2_SENSOR_GYRO_ODR);
    float mag_initial_time = 0.1; // configure with 10Hz ODR
    err = sensor_dev->sensor_imu->init(clock_actual_rate, accel_initial_time, gyro_initial_time, &sensor_dev->accel_actual_time, &sensor_dev->gyro_actual_time);
    sensor_dev->sensor_actual_time = MIN(sensor_dev->accel_actual_time, sensor_dev->gyro_actual_time);
#if SENSOR_IMU_SPI_EXISTS || MULTIPLE_SENSORS
    LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_dev->sensor_imu_dev->config.frequency / 1000000.0);
#endif
    LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / (double)sensor_dev->accel_actual_time);
    LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / (double)sensor_dev->gyro_actual_time);
    if (err < 0)
        return err;
    // 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
    if (sensor_dev->mag_available && sensor_dev->mag_enabled)
    {
#if SENSOR_DIRECT_MAG_EXISTS
        sensor_dev->sensor_imu->ext_setup(SENSOR_EXT_MODE_I2C_PASSTHROUGH, NULL); // reenable passthrough
#elif SENSOR_MAG_EXT_EXISTS
        sensor_dev->sensor_imu->ext_setup(SENSOR_EXT_MODE_I2CM_PROXY, NULL, sensor_dev->sensor_mag_dev.addr & 0x7F);
#endif
        err = sensor_dev->sensor_mag->init(mag_initial_time, &sensor_dev->mag_actual_time);
        sensor_dev->mag_interval = sensor_dev->mag_actual_time * 1000 - 2; // start attemping magnetometer reads before expected new sample, ask for each sample 2ms earlier
        sensor_dev->mag_manual = true;

#if SENSOR_MAG_EXT_EXISTS
        // try to switch to autonomous mode
        if (sensor_dev->sensor_imu->ext_setup(SENSOR_EXT_MODE_I2CM_AUTONOMOUS, sensor_mag, sensor_dev->sensor_mag_dev.addr & 0x7F) == 0)
        {
            // switched to autonomous, no need to handle mag manually
            mag_manual = false;
        }
#endif

#if SENSOR_MAG_SPI_EXISTS
        LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_dev->sensor_mag_spi_dev.config.frequency / 1000000.0);
#endif
        LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / (double)sensor_dev->mag_actual_time);
        if (err < 0)
            return err;
        // 0-1ms to setup mmc
    }
    LOG_INF("Initialized sensors");

    // get fusion from config
    sensor_dev->fusion_id = CLAMP(CONFIG_2_SETTINGS_READ(CONFIG_2_SENSOR_FUSION), 0, FUSION_COUNT - 1);
    sensor_dev->sensor_fusion = sensor_fusions[sensor_dev->fusion_id];

    // Setup fusion
    sensor_retained_read(sensor_dev); // TODO: useless
    if (sensor_dev->fusion_id == FUSION_VQF)
        vqf_update_sensor_ids(sensor_dev->sensor_imu_id);
    if (retained->fusion_id == sensor_dev->fusion_id) // Check if the retained fusion data is valid and matches the selected fusion
    {                                                 // Load state if the data is valid (fusion was initialized before)
        sensor_dev->sensor_fusion->load(retained->fusion_data);
        retained->fusion_id = 0; // Invalidate retained fusion data
        retained_update();
    }
    else
    {
        sensor_dev->sensor_fusion->init(sensor_dev->gyro_actual_time, sensor_dev->accel_actual_time, sensor_dev->mag_actual_time);
    }

    sensor_calibration_update_sensor_ids(sensor_dev->sensor_imu_id);
    if (sensor_dev->sensor_imu == &sensor_imu_bmi270) // bmi270 specific
    {
        LOG_INF("Applying gyroscope gain");
        bmi_gain_apply(sensor_calibration_get_sensor_data());
    }

    // Setup interrupt
    float fifo_threshold = sensor_dev->sensor_update_time_ms / 1000.0f / sensor_dev->sensor_actual_time; // target loop rate
    sensor_dev->sensor_fifo_threshold = fifo_threshold;
    LOG_DBG("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_dev->sensor_fifo_threshold);
    uint8_t pin_config = sensor_dev->sensor_imu->setup_DRDY(sensor_dev->sensor_fifo_threshold);
    if (pin_config == 0)
        return -1;
#if IMU_INT_EXISTS
    uint32_t int0_gpios = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios);
    LOG_DBG("FIFO THS/WM/WTM GPIO pin: %u, config: %u", int0_gpios, pin_config);
    uint32_t pull_flags = ((pin_config >> 4) == NRF_GPIO_PIN_PULLDOWN ? GPIO_PULL_DOWN : 0) | ((pin_config >> 4) == NRF_GPIO_PIN_PULLUP ? GPIO_PULL_UP : 0);
    gpio_pin_configure_dt(&int0, GPIO_INPUT | pull_flags);
    uint32_t int_flags = ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_LOW ? GPIO_INT_EDGE_FALLING : 0) | ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_HIGH ? GPIO_INT_EDGE_RISING : 0);
    gpio_pin_interrupt_configure_dt(&int0, int_flags);
    gpio_init_callback(&sensor_cb_data, sensor_interrupt_handler, BIT(int0.pin));
    gpio_add_callback(int0.port, &sensor_cb_data);
#else
    LOG_WRN("IMU FIFO THS/WM/WTM GPIO does not exist");
    LOG_WRN("IMU FIFO THS/WM/WTM not available");
#endif

    LOG_INF("Using %s", fusion_names[sensor_dev->fusion_id]);
    LOG_INF("Initialized fusion");
    sensor_dev->sensor_fusion_init = true;
    sensor_dev->packet_errors = 0;
    return 0;
}

static int sensor_scan()
{
    int err = 0;
    int retries = 5;
    sys_interface_resume(); // make sure interfaces are enabled
    // while (retries > 0)
    // {
    //     sensor_imu_dev.addr = 0x00;
    //     if (err == 0)
    //     {
    //         k_msleep(5);
    //         LOG_INF("Retrying sensor detection");

    //         // Reset address before retrying sensor detection
    //         sensor_imu_dev.addr = 0x00;

    //         err = sensor_scan(); // on POR, the sensor may not be ready yet
    //     }
    // }
    // sys_interface_suspend();
    if (err)
        return err;

    // sensor_scan_read();
    int imu_id = -1;

    for (int i = 0; i < IMU_COUNT; i++)
    {
        sensor_dev_t *sensor_dev = &connected_sensors[i];

        initialize_sensor_defaults(sensor_dev);
        sensor_dev->sensor_update_time_ms = 6;
        sensor_dev->sensor_imu_dev = &sensor_imu_spi_devs[i].spec;
        sensor_dev->sensor_position = sensor_imu_spi_devs[i].sensor_position;
        err = sensor_scan_imu_spi(&sensor_imu_spi_devs[i].spec, &sensor_imu_dev_reg);
        if (err >= 0)
        {
            LOG_INF("Sensor ID: %d", imu_ids[i]);
            // sensor_interface_register_sensor_imu_spi(&sensor_imu_spi_devs[i]);
            if (err >= (int)ARRAY_SIZE(dev_imu_names))
                LOG_WRN("Found unknown device");
            else if (err < 0)
                LOG_ERR("No IMU detected");
            else
                LOG_INF("Found %s", dev_imu_names[err]);
            imu_ids[i] = err;
            connected_sensors[i].sensor_imu = sensor_imus[imu_ids[i]];
            LOG_INF("Imu ID: %d, Count %d", imu_ids[i], i);
        }
        else
        {
            LOG_WRN("Imu %d not detected or incompatible!", i);
            continue;
        }
        // TODO: Finish mag implementation
        int mag_id = -1;
        sensor_scan_write();
        connection_update_sensor_ids(imu_id, mag_id);
        connected_sensors[i].sensor_imu_id = imu_ids[i];
        connected_sensors[i].sensor_mag_id = mag_id;

        connected_sensors[i].sensor_init = true;
        set_status(SYS_STATUS_SENSOR_ERROR, false);
    }
    sensor_loop_set_state(SENSOR_LOOP_INITIALIZING_IMU);
    return 0;
}

static int sensors_init()
{
    int err;
    for (int i = 0; i < IMU_COUNT; i++)
    {
        if (connected_sensors[i].sensor_imu_id != -1)
        {
            sensor_dev_t *sensor_dev = &connected_sensors[i];
            if (!sensor_dev->sensor_init)
                continue;
            sensor_interface_register_sensor_imu_spi(sensor_dev->sensor_imu_dev);

            err = sensor_init(&connected_sensors[i]);
            if (err)
            {
                LOG_ERR("Imu %d init failed: %d", i, err);
                sensor_dev->sensor_init = false;
                continue;
            }
            sensor_dev->main_ok = true;
        }
    }
    sensor_loop_set_state(SENSOR_LOOP_READING_IMU);
    return 0;
}

static int mags_init()
{
}

static int read_imus()
{
    int64_t last_status_time;
    int64_t read_pass_max;
    int err;
    int64_t read_pass_begin = 0;
    while (sensor_loop_state == SENSOR_LOOP_READING_IMU)
    {
        int64_t read_pass_begin = k_uptime_get();

        for (int i = 0; i < IMU_COUNT; i++)
        {
            if (!connected_sensors[i].main_ok)
            {
                continue;
            }
            sensor_interface_register_sensor_imu_spi(connected_sensors[i].sensor_imu_dev);
            bool reconfig = connected_sensors[i].last_sensor_mode != connected_sensors[i].sensor_mode;
            connected_sensors[i].last_sensor_mode = connected_sensors[i].sensor_mode;
            connected_sensors[i].time_begin = k_uptime_get();

            err = connected_sensors[i].sensor_imu->temp_read(&connected_sensors[i].temp);
            if (!err)
            {
                connected_sensors[i].last_temp_time = k_uptime_get();
                connection_update_sensor_temp(connected_sensors[i].temp);
            }

            uint16_t data_size = CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_LOW_POWER_2) ? 1900 : 1024; // Limit FIFO read to 2048 bytes (worst case is ICM 20 byte packet at 1000Hz and 100ms update time)
            uint8_t *raw_data = (uint8_t *)k_malloc(data_size);
            if (raw_data == NULL)
            {
                LOG_ERR("Failed to allocate memory for FIFO buffer");
                set_status(SYS_STATUS_SENSOR_ERROR, true);
                continue;
            }
            uint16_t packets = connected_sensors[i].sensor_imu->data_read(raw_data, data_size); // TODO: name this better?

            if (packets == 0)
            {
                k_free(raw_data);

                k_msleep(connected_sensors[i].sensor_update_time_ms > 0 ? connected_sensors[i].sensor_update_time_ms : 5);
                continue;
            }

            float raw_m[3];
            bool mag_read = false;
            if (connected_sensors[i].mag_manual && (k_uptime_get() - connected_sensors[i].last_mag_time > connected_sensors[i].mag_interval)) // some magnetometer do not have int pin // TODO: implement for magnetometer that does, or read status byte
            {
                mag_read = true;
                connected_sensors[i].sensor_mag->mag_read(raw_m); // reading mag last, and it will be processed last
            }

            int16_t last_sensor_fifo_threshold = connected_sensors[i].sensor_fifo_threshold;
            // if (reconfig) // TODO: get rid of reconfig?
            // {
            //     // Changing FIFO threshold here should be fine since FIFO is empty now
            //     switch (connected_sensors[i].sensor_mode)
            //     {
            //     case SENSOR_SENSOR_MODE_LOW_NOISE:
            //         set_update_time_ms(6, &connected_sensors[i]);
            //         LOG_INF("Switching sensors to low noise");
            //         break;
            //     case SENSOR_SENSOR_MODE_LOW_POWER:
            //         set_update_time_ms(33, &connected_sensors[i]);
            //         LOG_INF("Switching sensors to low power");
            //         break;
            //     case SENSOR_SENSOR_MODE_LOW_POWER_2:
            //         set_update_time_ms(100, &connected_sensors[i]);
            //         LOG_INF("Switching sensors to low power 2");
            //         break;
            //     };
            // }

            // Fuse all data
            int g_count = 0;
            float a_sum[3] = {0};
            int a_count = 0;
            int processed_packets = 0;

            process_imu_packets(connected_sensors[i], &packets, raw_data, raw_m, &g_count, &a_count, a_sum, &processed_packets, &mag_read);

            // If sensors have asymmetric packets in FIFO, timesteps will not match packet count
            int processed_timesteps = MAX(g_count, a_count);

            // Free the FIFO buffer
            k_free(raw_data);

            if (mag_read && memcmp(raw_m, connected_sensors[i].last_m, sizeof(connected_sensors[i].last_m))) // check data has changed from last acquisition
            {
                connected_sensors[i].last_mag_time = k_uptime_get();
                bool mag_calibrated = true;
                memcpy(connected_sensors[i].last_m, raw_m, sizeof(connected_sensors[i].last_m)); // copy raw magnetometer data
                sensor_calibration_process_mag(raw_m);
                float zero_m[3] = {0};
                if (v_epsilon(raw_m, zero_m, 1e-6)) // if the magnetometer is not calibrated, skip and send raw data
                {
                    memcpy(raw_m, connected_sensors[i].last_m, sizeof(connected_sensors[i].last_m));
                    mag_calibrated = false;
                }
                float mx = raw_m[0];
                float my = raw_m[1];
                float mz = raw_m[2];
                float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

                // Process fusion
                if (mag_calibrated)
                    connected_sensors[i].sensor_fusion->update_mag(m, connected_sensors[i].mag_actual_time);

                v_rotate(m, connected_sensors[i].q3, m); // magnetic field in local device frame, no other transformation will be done
                connection_update_sensor_mag(m);
            }

            // Copy average acceleration for this frame
            static float a[3] = {0}; // keep persistent
            if (a_count > 0)
            {
                for (int k = 0; k < 3; k++)
                    a[k] = a_sum[k] / a_count;
            }

            // Check packet processing
            if ((packets != 0 || k_uptime_get() > 100) && processed_packets == 0)
            {
                if (packets)
                    LOG_WRN("No packets processed");
                else
                    LOG_WRN("No packets in buffer");
                if (++connected_sensors[i].packet_errors == 10)
                {
                    LOG_ERR("Packet error threshold exceeded");
                    set_status(SYS_STATUS_SENSOR_ERROR, true);
                    if (packets)
                    {
                        sensor_retained_write(&connected_sensors[i]); // keep the fusion state
                        sys_request_system_reboot(false);
                    }
                }
            }
            else if (processed_packets == packets && packets > 0)
            {
                connected_sensors[i].packet_errors = 0;
            }

            // Also check if expected number of timesteps when using FIFO threshold, if FIFO threshold is being used
            if (last_sensor_fifo_threshold && processed_timesteps && processed_timesteps != last_sensor_fifo_threshold)
                LOG_WRN("Expected %d timestep%s, got %d", last_sensor_fifo_threshold, last_sensor_fifo_threshold == 1 ? "" : "s", processed_timesteps);

            // Update fusion gyro sanity? // TODO: use to detect drift and correct or suspend tracking
            //			sensor_fusion->update_gyro_sanity(g, m);

            connected_sensors[i].sensor_fusion->get_quat(connected_sensors[i].q);

            q_normalize(connected_sensors[i].q, connected_sensors[i].q); // safe to use self as output

            // Get linear acceleration
            float lin_a[3] = {0};
            if (v_diff_mag(a, lin_a) != 0) // lin_a as zero vector
                a_to_lin_a(connected_sensors[i].q, a, lin_a);

            sensor_update_sensor_state(&connected_sensors[i]);

            // Update orientation
            bool send_quat_data = !q_epsilon(connected_sensors[i].q, connected_sensors[i].last_q, 0.001);
            bool send_lin_accel_data = !v_epsilon(lin_a, connected_sensors[i].last_lin_a, 0.05);
            if (send_quat_data || send_lin_accel_data)
            {
                memcpy(connected_sensors[i].last_q, connected_sensors[i].q, sizeof(connected_sensors[i].q));
                memcpy(connected_sensors[i].last_lin_a, lin_a, sizeof(lin_a));
                float q_offset[4];
                q_multiply(connected_sensors[i].q, connected_sensors[i].q3, q_offset); // quaternion in device orientation, connection will change format from wxyz to xyzw
                v_rotate(lin_a, connected_sensors[i].q3, lin_a);                       // linear acceleration in local device frame, no other transformation will be done
                connection_update_sensor_data(q_offset, lin_a, connected_sensors[i].sensor_data_time, connected_sensors[i].sensor_position);
            }

            // Handle magnetometer calibration on transition
            if (connected_sensors[i].mag_available && connected_sensors[i].mag_enabled && connected_sensors[i].last_sensor_mode == SENSOR_SENSOR_MODE_LOW_NOISE && connected_sensors[i].sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER)
                sensor_request_calibration_mag();

            main_running = false;
            int64_t time_delta = k_uptime_get() - connected_sensors[i].time_begin;

            if (time_delta > connected_sensors[i].sensor_update_time_ms && time_delta > connected_sensors[i].max_loop_time)
                connected_sensors[i].max_loop_time = time_delta;
            if (k_uptime_get() - connected_sensors[i].last_status_time > STATUS_INTERVAL_MS)
            {
                connected_sensors[i].last_status_time = k_uptime_get();
                if (connected_sensors[i].max_loop_time > 0)
                {
                    LOG_WRN("Last update steps took up to %lld ms", connected_sensors[i].max_loop_time);
                    connected_sensors[i].max_loop_time = 0;
                }

#if IMU_INT_EXISTS
                sensor_data_time = 0; // reset data time
                if (!main_wfi)
                {
                    main_wfi = true;                      // TODO: this is terrible
                    k_msleep(sensor_update_time_ms + 10); // will be resumed by interrupt // TODO: dont use hard timeout
                    if (main_wfi)                         // timeout
                    {
                        LOG_WRN("Sensor interrupt timeout");
                        main_wfi = false;
                    }
                }
                else // if signal was sent during processing, loop immediately to catch up (I2C could cause this to happen constantly)
                {
                    LOG_DBG("Interrupt triggered during loop");
                    k_yield();
                    main_wfi = false;
                }
#else
                // TODO: old behavior
                //		led_clock_offset += time_delta;
                if (time_delta > connected_sensors[i].sensor_update_time_ms)
                    k_msleep(1);
                else
                    k_msleep(connected_sensors[i].sensor_update_time_ms - time_delta);
#endif
            }
        }
    }
    main_running = false;
    int64_t pass_time = k_uptime_get() - read_pass_begin;
    if (pass_time > read_pass_max)
        read_pass_max = pass_time;

    if (k_uptime_get() - last_status_time > STATUS_INTERVAL_MS)
    {
        last_status_time = k_uptime_get();
        if (read_pass_max > 6)
            LOG_WRN("Read pass took up to %lld ms", read_pass_max);
        read_pass_max = 0;
    }

    if (main_suspended) // TODO:
        k_thread_suspend(sensor_thread_id);

    main_running = true;
    return 0;
}

static int read_mags()
{
}

static void sensor_thread(void)
{
    int err;
    while (1)
    {

        // Scan sensor(s)

        // Initialize sensor(s)

        // Read sensor(s)

        switch (sensor_loop_state)
        {
        case SENSOR_LOOP_DISCOVERING:
            LOG_INF("Discovering sensors");
            err = sensor_scan();
            break;
        case SENSOR_LOOP_INITIALIZING_IMU:
            LOG_INF("Initializng IMUs");
            err = sensors_init();
            break;
        case SENSOR_LOOP_INITIALIZING_MAG:
            LOG_INF("Initializing Mags");
            mags_init();
            break;
        case SENSOR_LOOP_READING_IMU:
            LOG_INF("Reading IMUs");
            read_imus();
            break;
        case SENSOR_LOOP_READING_MAG:
            LOG_INF("Reading Mags");
            read_mags();
            break;
        }
    }
}