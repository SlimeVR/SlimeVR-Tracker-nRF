#include "interface.h"

#include <zephyr/logging/log.h>

//#define DEBUG true
//#define DEBUG_RATE true

#if DEBUG || DEBUG_RATE
LOG_MODULE_REGISTER(sensor_interface, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(sensor_interface, LOG_LEVEL_INF);
#endif

// TODO: move all sensor devices here? currently they are scanned by sensor loop

struct spi_dt_spec *sensor_interface_dev_spi[SENSOR_INTERFACE_DEV_COUNT];
struct i2c_dt_spec *sensor_interface_dev_i2c[SENSOR_INTERFACE_DEV_COUNT];
enum sensor_interface_spec sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_COUNT];

uint32_t sensor_interface_dev_spi_dummy_reads[SENSOR_INTERFACE_DEV_COUNT] = {0};

uint8_t ext_addr = 0x00;
uint8_t min_ext_burst = 0;
static const sensor_ext_ssi_t *ext_ssi = NULL;

// TODO: only one active spi transaction at a time
// TODO: spi burst read multiple buffers

uint8_t rx_tmp[8] = {0};
struct spi_buf tx_bufs[2];
struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
struct spi_buf rx_bufs[2];
struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};

// TODO: also keep reference to sensor device drivers (such as for ext mag)

void sensor_interface_register_sensor_imu_spi(struct spi_dt_spec *dev)
{
	sensor_interface_dev_spi[SENSOR_INTERFACE_DEV_IMU] = dev;
	sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_IMU] = SENSOR_INTERFACE_SPEC_SPI;
}

void sensor_interface_register_sensor_imu_i2c(struct i2c_dt_spec *dev)
{
	sensor_interface_dev_i2c[SENSOR_INTERFACE_DEV_IMU] = dev;
	sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_IMU] = SENSOR_INTERFACE_SPEC_I2C;
}

void sensor_interface_register_sensor_mag_spi(struct spi_dt_spec *dev)
{
	sensor_interface_dev_spi[SENSOR_INTERFACE_DEV_MAG] = dev;
	sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_MAG] = SENSOR_INTERFACE_SPEC_SPI;
}

void sensor_interface_register_sensor_mag_i2c(struct i2c_dt_spec *dev) // also used for passthrough
{
	sensor_interface_dev_i2c[SENSOR_INTERFACE_DEV_MAG] = dev;
	sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_MAG] = SENSOR_INTERFACE_SPEC_I2C;
}

int sensor_interface_register_sensor_mag_ext(uint8_t addr, uint8_t min_burst, uint8_t burst)
{
	switch (sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_IMU])
	{
	case SENSOR_INTERFACE_SPEC_SPI:
		if (ext_ssi != NULL)
		{
			if (burst > ext_ssi->ext_burst)
			{
				if (min_burst > ext_ssi->ext_burst)
				{
					LOG_ERR("Unsupported burst length");
					return -1;
				}
				LOG_WRN("Using minimum burst length");
			}
			min_ext_burst = min_burst; // fallback if num_read exceeds ext_burst
			ext_addr = addr;
			sensor_interface_dev_spec[SENSOR_INTERFACE_DEV_MAG] = SENSOR_INTERFACE_SPEC_EXT;
			return 0;
		}
		else
		{
			LOG_ERR("IMU must configure external interface before registering magnetometer");
			return -1;
		}
		break;
	case SENSOR_INTERFACE_SPEC_I2C:
		LOG_ERR("External interface not used over I2C");
		return -1;
	default:
		LOG_ERR("IMU must be registered before registering magnetometer");
	}
	return -1;
}

// must be called to set correct frequency
int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads)
{
	if (sensor_interface_dev_spec[dev] != SENSOR_INTERFACE_SPEC_SPI)
		return -1; // no spi device registered
	sensor_interface_dev_spi[dev]->config.frequency = frequency;
	sensor_interface_dev_spi_dummy_reads[dev] = dummy_reads; // shoutout to BMI270
	return 0;
}

void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext)
{
	ext_ssi = ext;
}

const sensor_ext_ssi_t *sensor_interface_ext_get(void)
{
	return ext_ssi;
}

// TODO: spi config by device

int ssi_write(enum sensor_interface_dev dev, const uint8_t *buf, uint32_t num_bytes)
{
	switch (sensor_interface_dev_spec[dev])
	{
	case SENSOR_INTERFACE_SPEC_SPI:
		tx_bufs[0].buf = (void *)buf;
		tx_bufs[0].len = num_bytes;
		tx.count = 1;
		k_usleep(1);
#if DEBUG
		LOG_DBG("ssi_write: dev=%d, num_bytes=%zu", dev, num_bytes);
		LOG_HEXDUMP_DBG(buf, num_bytes, "ssi_write: buf");
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
		LOG_DBG("ssi_write: err=%d", err);
		k_msleep(500);
		return err;
#elif DEBUG_RATE
		int64_t start = k_uptime_ticks();
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
		int64_t end = k_uptime_ticks();
		printk("ssi_write: %zuB, %.2f MB/s\n", num_bytes, (double)num_bytes / (double)k_ticks_to_us_near64(end - start));
		return err;
#else
		return spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
#endif
	case SENSOR_INTERFACE_SPEC_I2C:
		return i2c_write_dt(sensor_interface_dev_i2c[dev], buf, num_bytes);
  case SENSOR_INTERFACE_SPEC_EXT:
    if (ext_ssi != NULL)
      return ext_ssi->ext_write(ext_addr, buf, num_bytes);
	default:
		return -1;
	}
}

int ssi_read(enum sensor_interface_dev dev, uint8_t *buf, uint32_t num_bytes)
{
	switch (sensor_interface_dev_spec[dev])
	{
	case SENSOR_INTERFACE_SPEC_SPI:
		// TODO: this may be zero!
		rx_bufs[0].buf = rx_tmp;
		rx_bufs[0].len = sensor_interface_dev_spi_dummy_reads[dev];
		rx_bufs[1].buf = buf;
		rx_bufs[1].len = num_bytes;
		rx.count = 2;
		k_usleep(1);
#if DEBUG
		LOG_DBG("ssi_read: dev=%d, num_bytes=%zu", dev, num_bytes);
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], NULL, &rx);
		LOG_HEXDUMP_DBG(rx_tmp, sensor_interface_dev_spi_dummy_reads[dev], "ssi_read: rx_tmp");
		LOG_HEXDUMP_DBG(buf, num_bytes, "ssi_read: buf");
		LOG_DBG("ssi_read: err=%d", err);
		k_msleep(500);
		return err;
#elif DEBUG_RATE
		int64_t start = k_uptime_ticks();
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], NULL, &rx);
		int64_t end = k_uptime_ticks();
		printk("ssi_read: %zuB, %.2f MB/s\n", num_bytes, (double)num_bytes / (double)k_ticks_to_us_near64(end - start));
		return err;
#else
		return spi_transceive_dt(sensor_interface_dev_spi[dev], NULL, &rx);
#endif
	case SENSOR_INTERFACE_SPEC_I2C:
		return i2c_read_dt(sensor_interface_dev_i2c[dev], buf, num_bytes);
	default:
		return -1;
	}
}

int ssi_write_read(enum sensor_interface_dev dev, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	// TODO: is separate read/write better for spi?
	switch (sensor_interface_dev_spec[dev])
	{
	case SENSOR_INTERFACE_SPEC_SPI:
		tx_bufs[0].buf = (void *)write_buf;
		tx_bufs[0].len = num_write;
		tx.count = 1;
		rx_bufs[0].buf = rx_tmp;
		rx_bufs[0].len = num_write + sensor_interface_dev_spi_dummy_reads[dev];
		rx_bufs[1].buf = read_buf;
		rx_bufs[1].len = num_read;
		rx.count = 2;
		k_usleep(1);
#if DEBUG
		LOG_DBG("ssi_write_read: dev=%d, num_write=%zu, num_read=%zu", dev, num_write, num_read);
		LOG_HEXDUMP_DBG(write_buf, num_write, "ssi_write_read: write_buf");
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, &rx);
		LOG_HEXDUMP_DBG(rx_tmp, num_write + sensor_interface_dev_spi_dummy_reads[dev], "ssi_write_read: rx_tmp");
		LOG_HEXDUMP_DBG(read_buf, num_read, "ssi_write_read: read_buf");
		LOG_DBG("ssi_write_read: err=%d", err);
		k_msleep(500);
		return err;
#elif DEBUG_RATE
		int64_t start = k_uptime_ticks();
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, &rx);
		int64_t end = k_uptime_ticks();
		printk("ssi_write_read: %zuB, %.2f MB/s\n", (num_write + num_read), (double)(num_write + num_read) / (double)k_ticks_to_us_near64(end - start));
		return err;
#else
		return spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, &rx);
#endif
	case SENSOR_INTERFACE_SPEC_I2C:
		return i2c_write_read_dt(sensor_interface_dev_i2c[dev], write_buf, num_write, read_buf, num_read);
	case SENSOR_INTERFACE_SPEC_EXT:
		if (ext_ssi != NULL)
		{
			if (num_read > ext_ssi->ext_burst)
				num_read = min_ext_burst;
			return ext_ssi->ext_write_read(ext_addr, write_buf, num_write, read_buf, num_read);
		}
	default:
		return -1;
	}
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes)
{
	if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI)
		start_addr |= 0x80; // set read bit
	return ssi_write_read(dev, &start_addr, 1, buf, num_bytes);
}

int ssi_burst_write(enum sensor_interface_dev dev, uint8_t start_addr, const uint8_t *buf, uint32_t num_bytes)
{
	switch (sensor_interface_dev_spec[dev])
	{
	case SENSOR_INTERFACE_SPEC_SPI:
		tx_bufs[0].buf = &start_addr;
		tx_bufs[0].len = 1;
		tx_bufs[1].buf = (void *)buf;
		tx_bufs[1].len = num_bytes;
		tx.count = 2;
		k_usleep(1);
#if DEBUG
		LOG_DBG("ssi_burst_write: dev=%d, start_addr=0x%02X, num_bytes=%d", dev, start_addr, num_bytes);
		LOG_HEXDUMP_DBG(&start_addr, 1, "ssi_burst_write: start_addr");
		LOG_HEXDUMP_DBG(buf, num_bytes, "ssi_burst_write: buf");
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
		LOG_DBG("ssi_burst_write: err=%d", err);
		k_msleep(500);
		return err;
#elif DEBUG_RATE
		int64_t start = k_uptime_ticks();
		int err = spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
		int64_t end = k_uptime_ticks();
		printk("ssi_burst_write: %zuB, %.2f MB/s\n", num_bytes, (double)num_bytes / (double)k_ticks_to_us_near64(end - start));
		return err;
#else
		return spi_transceive_dt(sensor_interface_dev_spi[dev], &tx, NULL);
#endif
	case SENSOR_INTERFACE_SPEC_I2C:
		return i2c_burst_write_dt(sensor_interface_dev_i2c[dev], start_addr, buf, num_bytes);
	default:
		return -1;
	}
}

int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t *value)
{
	if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI)
		reg_addr |= 0x80; // set read bit
	return ssi_write_read(dev, &reg_addr, 1, value, 1);
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t value)
{
	uint8_t buf[2] = {reg_addr, value};
	return ssi_write(dev, buf, 2);
}

int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
	uint8_t old_value, new_value;
	if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI)
		reg_addr |= 0x80; // set read bit
	int err = ssi_reg_read_byte(dev, reg_addr, &old_value);
	if (err)
		return err;
	new_value = (old_value & ~mask) | (value & mask);
	if (new_value == old_value) {
		return 0;
	}
	if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI)
		reg_addr &= 0x7f; // clear read bit
	return ssi_reg_write_byte(dev, reg_addr, new_value);
}

int ssi_reg_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval)
{
#if DEBUG || DEBUG_RATE
	uint32_t start = k_cycle_get_32();
#endif
	// TODO: better way to handle with spi?
	// TODO: not working
	if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI)
		start_addr |= 0x80; // set read bit
	int err = ssi_write(dev, &start_addr, 1); // Start read buffer
//	if (err)
//		return err;
#if CONFIG_SOC_NRF52832
	uint32_t maxcnt = 255; // easyeda-maxcnt-bits = <8>
#elif CONFIG_SOC_NRF52810
	uint32_t maxcnt = 1023; // easyeda-maxcnt-bits = <10>
#else
	uint32_t maxcnt = 2048; // all other SOC have >11 bits
#endif
	interval *= maxcnt / interval; // maximum interval below maxcnt
	while (num_bytes > 0)
	{
#if DEBUG || DEBUG_RATE
		LOG_DBG("ssi_reg_read_interval: num_bytes=%u", num_bytes);
#endif
		if (interval > num_bytes)
			interval = num_bytes;
		err |= ssi_read(dev, buf, interval);
//		if (err)
//			return err;
		buf += interval;
		num_bytes -= interval;
	}
#if DEBUG || DEBUG_RATE
	uint32_t end = k_cycle_get_32();
	LOG_DBG("ssi_reg_read_interval: us=%u", k_cyc_to_us_near32(end - start));
#endif
	return err;
}

int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval)
{
#if DEBUG || DEBUG_RATE
	uint32_t start = k_cycle_get_32();
#endif
	int err = 0;
#if CONFIG_SOC_NRF52832
	uint32_t maxcnt = 255; // easyeda-maxcnt-bits = <8>
#elif CONFIG_SOC_NRF52810
	uint32_t maxcnt = 1023; // easyeda-maxcnt-bits = <10>, I2C timeout (>25ms) on higher interval
#else
	uint32_t maxcnt = sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI ? 16383 : 1023; // all other SOC have >=14 bits, I2C timeout (>25ms) on higher interval
#endif
	interval *= maxcnt / interval; // maximum interval below maxcnt
	while (num_bytes > 0)
	{
#if DEBUG || DEBUG_RATE
		LOG_DBG("ssi_burst_read_interval: num_bytes=%u", num_bytes);
#endif
		if (interval > num_bytes)
			interval = num_bytes;
		err |= ssi_burst_read(dev, start_addr, buf, interval);
//		if (err)
//			return err;
		buf += interval;
		num_bytes -= interval;
	}
#if DEBUG || DEBUG_RATE
	uint32_t end = k_cycle_get_32();
	LOG_DBG("ssi_burst_read_interval: us=%u", k_cyc_to_us_near32(end - start));
#endif
	return err;
}
