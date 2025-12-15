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
#include "sensor/sensors_enum.h"
#include "system/status.h"

#include "app_version.h"

#ifdef CONFIG_NCS_APPLICATION_BOOT_BANNER_STRING
#define FW_NAME CONFIG_NCS_APPLICATION_BOOT_BANNER_STRING
#else
#define FW_NAME "SlimeNRF Tracker"
#endif
#define FW_VERSION_MAJOR APP_VERSION_MAJOR
#define FW_VERSION_MINOR APP_VERSION_MINOR
#define FW_VERSION_PATCH APP_PATCHLEVEL

static uint8_t get_server_constant_imu_id(int id) __attribute__((unused));
static uint8_t get_server_constant_mag_id(int id) __attribute__((unused));
static uint8_t get_server_constant_tracker_status(int status) __attribute__((unused));

// constants from server, should include BoardType, MCUType, IMUType, MagType (not yet)
// https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/FirmwareConstants.kt
#define SVR_IMU_UNKNOWN 0
#define SVR_IMU_MPU9250 1
#define SVR_IMU_MPU6500 2
#define SVR_IMU_BNO080 3
#define SVR_IMU_BNO085 4
#define SVR_IMU_BNO055 5
#define SVR_IMU_MPU6050 6
#define SVR_IMU_BNO086 7
#define SVR_IMU_BMI160 8
#define SVR_IMU_ICM20948 9
#define SVR_IMU_ICM42688 10
#define SVR_IMU_BMI270 11
#define SVR_IMU_LSM6DS3TRC 12
#define SVR_IMU_LSM6DSV 13
#define SVR_IMU_LSM6DSO 14
#define SVR_IMU_LSM6DSR 15
#define SVR_IMU_ICM45686 16
#define SVR_IMU_ICM45605 17
#define SVR_IMU_ADC_RESISTANCE 18
#define SVR_IMU_DEV_RESERVED 250

#define SVR_BOARD_UNKNOWN 0
#define SVR_BOARD_SLIMEVR_LEGACY 1
#define SVR_BOARD_SLIMEVR_DEV 2
#define SVR_BOARD_NODEMCU 3
#define SVR_BOARD_CUSTOM 4
#define SVR_BOARD_WROOM32 5
#define SVR_BOARD_WEMOSD1MINI 6
#define SVR_BOARD_TTGO_TBASE 7
#define SVR_BOARD_ESP01 8
#define SVR_BOARD_SLIMEVR 9
#define SVR_BOARD_LOLIN_C3_MINI 10
#define SVR_BOARD_BEETLE32C32 11
#define SVR_BOARD_ES32C3DEVKITM1 12
#define SVR_BOARD_OWOTRACK 13
#define SVR_BOARD_WRANGLER 14
#define SVR_BOARD_MOCOPI 15
#define SVR_BOARD_WEMOSWROOM02 16
#define SVR_BOARD_XIAO_ESP32C3 17
#define SVR_BOARD_HARITORA 18
#define SVR_BOARD_ESP32C6DEVKITC1 19
#define SVR_BOARD_GLOVE_IMU_SLIMEVR_DEV 20
#define SVR_BOARD_GESTURES 21
#define SVR_BOARD_SLIMEVR_V1_2 22
#define SVR_BOARD_ESP32S3_SUPERMINI 23
#define SVR_BOARD_GENERIC_NRF 24
#define SVR_BOARD_SLIMEVR_BUTTERFLY_DEV 25
#define SVR_BOARD_SLIMEVR_BUTTERFLY 26
#define SVR_BOARD_DEV_RESERVED 250

#define SVR_MCU_UNKNOWN 0
#define SVR_MCU_ESP8266 1
#define SVR_MCU_ESP32 2
#define SVR_MCU_OWOTRACK_ANDROID 3
#define SVR_MCU_WRANGLER 4
#define SVR_MCU_OWOTRACK_IOS 5
#define SVR_MCU_ESP32_C3 6
#define SVR_MCU_MOCOPI 7
#define SVR_MCU_HARITORA 8
#define SVR_MCU_NRF52 9
#define SVR_MCU_NRF54L 10
#define SVR_MCU_DEV_RESERVED 250

#define SVR_MAG_STATUS_NOT_SUPPORTED 0
#define SVR_MAG_STATUS_DISABLED 1
#define SVR_MAG_STATUS_ENABLED 2

// https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/TrackerStatus.kt
#define SVR_STATUS_DISCONNECTED 0
#define SVR_STATUS_OK 1
#define SVR_STATUS_BUSY 2
#define SVR_STATUS_ERROR 3
#define SVR_STATUS_OCCLUDED 4
#define SVR_STATUS_TIMED_OUT 5

#if CONFIG_BOARD_SLIMEVRMINI_P1_UF2 || CONFIG_BOARD_SLIMEVRMINI_P2_UF2 || CONFIG_BOARD_SLIMEVRMINI_P3R6_UF2 || CONFIG_BOARD_SLIMEVRMINI_P3R7_UF2 || CONFIG_BOARD_SLIMEVRMINI_P4_UF2 || CONFIG_BOARD_SLIMEVRMINI_P4R9_UF2
#define FW_BOARD SVR_BOARD_SLIMEVR_BUTTERFLY_DEV
#else
#define FW_BOARD SVR_BOARD_GENERIC_NRF
#endif

#if CONFIG_SOC_SERIES_NRF52X
#define FW_MCU SVR_MCU_NRF52
#elif CONFIG_SOC_SERIES_NRF54LX
#define FW_MCU SVR_MCU_NRF54L
#else
#define FW_MCU 0
#endif

static uint8_t get_server_constant_imu_id(int id)
{
	switch (id)
	{
	case IMU_BMI160:
		return SVR_IMU_BMI160;
	case IMU_BMI270:
		return SVR_IMU_BMI270;
	case IMU_BMI323:
		return 0;
	case IMU_MPU6050:
		return SVR_IMU_MPU6050;
	case IMU_MPU6500:
		return SVR_IMU_MPU6500;
	case IMU_MPU9250:
		return SVR_IMU_MPU9250;
	case IMU_ICM20948:
		return SVR_IMU_ICM20948;
	case IMU_ICM42688:
		return SVR_IMU_ICM42688;
	case IMU_ICM45686:
		return SVR_IMU_ICM45686;
	case IMU_ISM330IS:
		return 0;
	case IMU_LSM6DS3:
		return 0;
	case IMU_LSM6DSM:
		return SVR_IMU_LSM6DS3TRC;
	case IMU_LSM6DSR:
		return SVR_IMU_LSM6DSR;
	case IMU_LSM6DSO:
		return SVR_IMU_LSM6DSO;
	case IMU_LSM6DST:
		return 0;
	case IMU_LSM6DSV:
		return SVR_IMU_LSM6DSV;
	case IMU_ISM330BX:
		return SVR_IMU_LSM6DSV; // not really
	default:
		return SVR_IMU_UNKNOWN;
	}
}

// does not exist in server enums yet
static uint8_t get_server_constant_mag_id(int id)
{
	return SVR_MAG_STATUS_NOT_SUPPORTED;
//	switch (id)
//	{
//	case MAG_HMC5883L:
//		return 0;
//	case MAG_QMC5883L:
//		return 0;
//	case MAG_AK8963:
//		return 0;
//	case MAG_AK09916:
//		return 0;
//	case MAG_AK09940:
//		return 0;
//	case MAG_BMM150:
//		return 0;
//	case MAG_BMM350:
//		return 0;
//	case MAG_IST8306:
//		return 0;
//	case MAG_IST8308:
//		return 0;
//	case MAG_IST8320:
//		return 0;
//	case MAG_IST8321:
//		return 0;
//	case MAG_LIS2MDL:
//		return 0;
//	case MAG_LIS3MDL:
//		return 0;
//	case MAG_MMC34160PJ:
//		return 0;
//	case MAG_MMC3630KJ:
//		return 0;
//	case MAG_MMC5633NJL:
//		return 0;
//	case MAG_MMC5616WA:
//		return 0;
//	case MAG_MMC5983MA:
//		return 0;
//	default:
//		return 0;
//	}
}

static uint8_t get_server_constant_tracker_status(int status)
{
	if (status & (SYS_STATUS_SENSOR_ERROR | SYS_STATUS_SYSTEM_ERROR))
		return SVR_STATUS_ERROR;
	else
		return SVR_STATUS_OK;
}

// https://stackoverflow.com/questions/11697820/how-to-use-date-and-time-predefined-macros-in-as-two-integers-then-stri
#define COMPUTE_BUILD_YEAR \
	( \
		(__DATE__[ 7] - '0') * 1000 + \
		(__DATE__[ 8] - '0') *  100 + \
		(__DATE__[ 9] - '0') *   10 + \
		(__DATE__[10] - '0') \
	)

#define COMPUTE_BUILD_DAY \
	( \
		((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + \
		(__DATE__[5] - '0') \
	)

#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')

#define COMPUTE_BUILD_MONTH \
	( \
		(BUILD_MONTH_IS_JAN) ?  1 : \
		(BUILD_MONTH_IS_FEB) ?  2 : \
		(BUILD_MONTH_IS_MAR) ?  3 : \
		(BUILD_MONTH_IS_APR) ?  4 : \
		(BUILD_MONTH_IS_MAY) ?  5 : \
		(BUILD_MONTH_IS_JUN) ?  6 : \
		(BUILD_MONTH_IS_JUL) ?  7 : \
		(BUILD_MONTH_IS_AUG) ?  8 : \
		(BUILD_MONTH_IS_SEP) ?  9 : \
		(BUILD_MONTH_IS_OCT) ? 10 : \
		(BUILD_MONTH_IS_NOV) ? 11 : \
		(BUILD_MONTH_IS_DEC) ? 12 : \
		/* error default */  99 \
	)

#define COMPUTE_BUILD_HOUR ((__TIME__[0] - '0') * 10 + __TIME__[1] - '0')
#define COMPUTE_BUILD_MIN  ((__TIME__[3] - '0') * 10 + __TIME__[4] - '0')
#define COMPUTE_BUILD_SEC  ((__TIME__[6] - '0') * 10 + __TIME__[7] - '0')

#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')

#define BUILD_YEAR  ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
#define BUILD_DAY   ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)

#define BUILD_TIME_IS_BAD (__TIME__[0] == '?')

#define BUILD_HOUR  ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_HOUR)
#define BUILD_MIN   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_MIN)
#define BUILD_SEC   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_SEC)

#define BUILD_TIMESTAMP (((((((BUILD_YEAR - 2020) & 127) * 12 + (BUILD_MONTH & 15)) * 31 + (BUILD_DAY & 31)) * 24 + (BUILD_HOUR & 24)) * 60 + (BUILD_MIN & 60)) * 60 + (BUILD_SEC & 60))

#define TOSTRING(x) STRINGIFY(x)

#define FW_STRING FW_NAME " " APP_VERSION_EXTENDED_STRING " "\
	"(Commit " TOSTRING(APP_BUILD_VERSION) ", Build %d-%02d-%02d %02d:%02d:%02d)\n",\
	BUILD_YEAR, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN, BUILD_SEC
