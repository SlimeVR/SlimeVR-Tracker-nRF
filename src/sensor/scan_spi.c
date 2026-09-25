/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(sensor_scan_spi, LOG_LEVEL_INF);

int sensor_scan_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg, int dev_addr_count, const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	uint8_t buf[3] = {0};
	uint8_t tx_data[3] = {0};
	struct spi_buf tx_buf = {.buf = tx_data, .len = 3};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = buf, .len = 3};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	int reg_index = 0;
	int id_index = 0;
	int found_id = 0;

	for (int i = 0; i < dev_addr_count; i++)
	{
		int reg_count = dev_reg[reg_index];
		int id_count = dev_id[id_index];
		reg_index++;
		id_index++;
		int id_cnt = id_count;
		int id_ind = id_index;
		int fnd_id = found_id;
		for (int k = 0; k < reg_count; k++)
		{
			uint8_t reg = dev_reg[reg_index + k];
			if (*spi_dev_reg == 0xFF || *spi_dev_reg == reg)
			{
				uint8_t id;
				tx_data[0] = reg | 0x80; // set read bit
				LOG_DBG("Scanning register: 0x%02X", reg);
				// TODO: BMM150 workaround?
				int err = spi_transceive_dt(bus, &tx, &rx);
				LOG_DBG("err: %d", err);
				id = buf[1] ? buf[1] : buf[2]; // ID may be in first byte, or skip one byte (such as BMI270)
				LOG_DBG("Read value: 0x%02X, 0x%02X, 0x%02X (0x%02X)", buf[0], buf[1], buf[2], id);
				if (err)
					continue;
				for (int l = 0; l < id_cnt; l++)
				{
					if (id == dev_id[id_ind + l])
					{
						*spi_dev_reg = reg;
						LOG_INF("Valid device found using register: 0x%02X (value: 0x%02X)", reg, id);
						return dev_ids[fnd_id + l];
					}
				}
			}
			id_ind += id_cnt;
			fnd_id += id_cnt;
			id_cnt = dev_id[id_ind];
			id_ind++;
		}
		reg_index += reg_count;
		id_index += id_count;
		found_id += id_count;
		for (int j = 1; j < reg_count; j++)
		{
			id_count = dev_id[id_index];
			id_index++;
			id_index += id_count;
			found_id += id_count;
		}
	}

	if (*spi_dev_reg != 0xFF) // preferred register failed, try again with full scan
	{
		LOG_WRN("No device found using register: 0x%02X", *spi_dev_reg);
		*spi_dev_reg = 0xFF;
		k_msleep(50);
		return sensor_scan_spi(bus, spi_dev_reg, dev_addr_count, dev_reg, dev_id, dev_ids);
	}

	return -1;
}