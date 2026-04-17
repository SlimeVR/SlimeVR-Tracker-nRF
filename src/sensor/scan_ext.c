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

#include "interface.h"

#define SCAN_ADDR_START 8
#define SCAN_ADDR_STOP 119

LOG_MODULE_REGISTER(sensor_scan_ext, LOG_LEVEL_DBG);

int sensor_scan_ext(const sensor_ext_ssi_t *ext_ssi, uint16_t *ext_dev_addr, uint8_t *ext_dev_reg, int dev_addr_count, const uint8_t dev_addr[], const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	if (*ext_dev_addr >= 0x7F) // ignoring device
		return -1;

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
			if (*ext_dev_addr >= SCAN_ADDR_START && *ext_dev_addr <= SCAN_ADDR_STOP && addr != *ext_dev_addr)
				continue; // if an address was provided try to scan it first
			LOG_DBG("Scanning address: 0x%02X", addr);

			int id_cnt = id_count;
			int id_ind = id_index;
			int fnd_id = found_id;
			for (int k = 0; k < reg_count; k++)
			{
				uint8_t reg = dev_reg[reg_index + k];
				if (*ext_dev_reg == 0xFF || *ext_dev_reg == reg)
				{
					int err;
					uint8_t id;
					LOG_DBG("Scanning register: 0x%02X", reg);
					if (reg == 0x40 && addr >= 0x10 && addr <= 0x13) // edge case for BMM150
					{
						int err = ext_ssi->ext_write(addr, (const uint8_t[]){0x4B, 0x01}, 2); // BMM150 cannot read chip id without power control enabled
						if (err)
							break;
						LOG_DBG("Power up BMM150");
						k_msleep(2); // BMM150 start-up
					}
					if (reg == 0x00 && addr >= 0x14 && addr <= 0x17) // edge case for BMM350
					{
						uint8_t buf[3];
						err = ext_ssi->ext_write_read(addr, &reg, 1, buf, 3); // BMM350 has two dummy bytes on read
						id = buf[2];
					}
					else
					{
						err = ext_ssi->ext_write_read(addr, &reg, 1, &id, 1);
					}
					LOG_DBG("Read value: 0x%02X", id);
					if (err)
						break;
					for (int l = 0; l < id_cnt; l++)
					{
						if (id == dev_id[id_ind + l])
						{
							*ext_dev_addr = addr;
							*ext_dev_reg = reg;
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

	if ((*ext_dev_addr >= SCAN_ADDR_START && *ext_dev_addr <= SCAN_ADDR_STOP) || *ext_dev_reg != 0xFF) // preferred address or register failed, try again with full scan
	{
		LOG_WRN("No device found at address: 0x%02X", *ext_dev_addr);
		*ext_dev_addr = 0;
		*ext_dev_reg = 0xFF;
		return sensor_scan_ext(ext_ssi, ext_dev_addr, ext_dev_reg, dev_addr_count, dev_addr, dev_reg, dev_id, dev_ids);
	}

	*ext_dev_addr = 0xFF; // no device found, mark as ignored
	return -1;
}
