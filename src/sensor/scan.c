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
#include <zephyr/drivers/i2c.h>

#define SCAN_ADDR_START 8
#define SCAN_ADDR_STOP 119

LOG_MODULE_REGISTER(sensor_scan, LOG_LEVEL_INF);

// Any address out of range (00, 7f, etc.) will search all addresses, otherwise it will check provided address and register first

int sensor_scan_i2c(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg, int dev_addr_count, const uint8_t dev_addr[], const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	if (i2c_dev->addr >= 0x7F) // ignoring device
	{
//		i2c_dev->addr = 0xFF; // no device found, mark as ignored
		return -1;
	}

	const struct device *dev = i2c_dev->bus;

	uint16_t addr = 0;

	int addr_index = 0;
	int reg_index = 0;
	int id_index = 0;
	int found_id = 0;

	for (int i = 0; i < dev_addr_count; i++)
	{
		int addr_count = dev_addr[addr_index];
		int reg_count = dev_reg[reg_index];
		int id_count = dev_id[id_index];
		addr_index++;
		reg_index++;
		id_index++;
		for (int j = 0; j < addr_count; j++)
		{
			addr = dev_addr[addr_index + j];
//			if (i2c_dev->addr >= SCAN_ADDR_START && i2c_dev->addr <= SCAN_ADDR_STOP && addr < i2c_dev->addr)
			if (i2c_dev->addr >= SCAN_ADDR_START && i2c_dev->addr <= SCAN_ADDR_STOP && addr != i2c_dev->addr)
				continue; // if an address was provided try to scan it first
			LOG_DBG("Scanning address: 0x%02X", addr);

			// The first read on ICM-45686 can fail, so perform a dummy read on each address first
			/* AN-000364
			 * In I2C mode, after chip power-up, the host should perform one retry
			 * on the very first I2C transaction if it receives a NACK
			 */
			uint8_t dummy;
			i2c_reg_read_byte(dev, addr, 0x00, &dummy);

			// QMC5883P wakeup - device won't respond in some cases
			if (addr == 0x2C)
			{
				uint8_t wake_seq[2] = { 0x09, 0x01 };
				int err = i2c_write(dev, wake_seq, sizeof(wake_seq), addr);
				if (!err)
				{
					LOG_INF("QMC5883P wakeup sequence sent to 0x2C");
					k_msleep(10);
					uint8_t id_byte = 0;
					if (i2c_reg_read_byte(dev, addr, 0x00, &id_byte) == 0)
						LOG_INF("QMC5883P probe: WHOAMI=0x%02X at 0x2C", id_byte);
				}
				else
				{
					LOG_WRN("Failed to wake QMC5883P at 0x2C (err=%d)", err);
				}
			}

			int id_cnt = id_count;
			int id_ind = id_index;
			int fnd_id = found_id;
			for (int k = 0; k < reg_count; k++)
			{
				uint8_t reg = dev_reg[reg_index + k];
				if (*i2c_dev_reg == 0xFF || *i2c_dev_reg == reg)
				{
					uint8_t id;
					LOG_DBG("Scanning register: 0x%02X", reg);
					if (reg == 0x40 && addr >= 0x10 && addr <= 0x13) // edge case for BMM150
					{
						int err = i2c_reg_write_byte(dev, addr, 0x4B, 0x01); // BMM150 cannot read chip id without power control enabled
						if (err)
							break;
						LOG_DBG("Power up BMM150");
						k_msleep(2); // BMM150 start-up
					}
					int err = i2c_reg_read_byte(dev, addr, reg, &id);
					LOG_DBG("Read value: 0x%02X", id);
					if (err)
						break;
					for (int l = 0; l < id_cnt; l++)
					{
						if (id == dev_id[id_ind + l])
						{
							i2c_dev->addr = addr;
							*i2c_dev_reg = reg;
							LOG_INF("Valid device found at address: 0x%02X (register: 0x%02X, value: 0x%02X)", addr, reg, id);
							return dev_ids[fnd_id + l];
						}
					}
				}
				id_ind += id_cnt;
				fnd_id += id_cnt;
				id_cnt = dev_id[id_ind];
				id_ind++;
			}
		}
		addr_index += addr_count;
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

	if ((i2c_dev->addr >= SCAN_ADDR_START && i2c_dev->addr <= SCAN_ADDR_STOP) || *i2c_dev_reg != 0xFF) // preferred address or register failed, try again with full scan
	{
		LOG_WRN("No device found at address: 0x%02X", i2c_dev->addr);
		i2c_dev->addr = 0;
		*i2c_dev_reg = 0xFF;
		return sensor_scan_i2c(i2c_dev, i2c_dev_reg, dev_addr_count, dev_addr, dev_reg, dev_id, dev_ids);
	}

	i2c_dev->addr = 0xFF; // no device found, mark as ignored
	return -1;
}
