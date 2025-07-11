#include "globals.h"
#include "system/system.h"

#include <zephyr/kernel.h>

#include "battery_tracker.h"

//#define DEBUG true

LOG_MODULE_REGISTER(battery_tracker, LOG_LEVEL_INF);

struct battery_tracker
{
	int16_t last_max_battery_pptt;
	int16_t last_min_battery_pptt;
	uint64_t last_battery_runtime;
};

struct battery_tracker_interval
{
	uint16_t cycles;
	uint64_t runtime;
	uint64_t runtime_min;
	uint64_t runtime_max;
};

static void update_statistics(void)
{
	if (retained->max_battery_pptt - retained->min_battery_pptt < 1000)
		return;

	// save last statistics
	struct battery_tracker tracker;
	tracker.last_max_battery_pptt = retained->max_battery_pptt;
	tracker.last_min_battery_pptt = retained->min_battery_pptt;
	tracker.last_battery_runtime = retained->battery_runtime_sum;

	sys_write(BATT_STATS_LAST_RUN_ID, NULL, &tracker, sizeof(tracker));
	LOG_DBG("Discharge: %6.2f%% -> %5.2f%%, %llu us", (double)tracker.last_max_battery_pptt / 100.0, (double)tracker.last_min_battery_pptt / 100.0, k_ticks_to_us_floor64(tracker.last_battery_runtime));
}

static void update_runtime(void)
{
	uint64_t now = k_uptime_ticks();
	retained->battery_runtime_sum += (now - retained->battery_uptime_latest);
	retained->battery_uptime_latest = now;
}

static void reset_tracker(int16_t pptt)
{
	update_runtime(); // update battery_runtime_sum before resetting

	retained->max_battery_pptt = pptt; // reset
	retained->min_battery_pptt = pptt;
	retained->battery_runtime_sum = 0;
	retained->battery_runtime_saved = 0;
	retained->battery_pptt_saved = (pptt + 499) / 500 * 500;
	if (pptt >= 0)
		LOG_DBG("Reset battery tracker, start tracking below %.2f%% (valid below %.2f%%)", (double)(retained->battery_pptt_saved - 500) / 100.0, (double)(pptt - 300) / 100.0);
	else
		LOG_DBG("Reset battery tracker");
}

static void update_interval(int16_t pptt)
{
	update_runtime(); // update battery_runtime_sum before saving

	uint8_t interval_id = (pptt + 499) / 500;
	uint64_t runtime = retained->battery_runtime_sum - retained->battery_runtime_saved;
	if (runtime < CONFIG_SYS_CLOCK_TICKS_PER_SEC * 300)
	{
		LOG_ERR("Interval %u: %llu us is too short", interval_id, k_ticks_to_us_floor64(runtime));
		return;
	}

	struct battery_tracker_interval interval;
	sys_read(BATT_STATS_INTERVAL_0 + interval_id, &interval, sizeof(interval));

	// TODO: can use nvs_read_hist
	interval.cycles++;
	interval.runtime += runtime;
	if (runtime < interval.runtime_min || interval.runtime_min == 0)
		interval.runtime_min = runtime;
	if (runtime > interval.runtime_max)
		interval.runtime_max = runtime;
	sys_write(BATT_STATS_INTERVAL_0 + interval_id, NULL, &interval, sizeof(interval));
	LOG_DBG("Interval %u: %u cycles, %llu us (current: %llu us, min: %llu us, max: %llu us)", interval_id, interval.cycles, k_ticks_to_us_floor64(interval.runtime), k_ticks_to_us_floor64(runtime), k_ticks_to_us_floor64(interval.runtime_min), k_ticks_to_us_floor64(interval.runtime_max));
}

static void update_tracker(int16_t pptt)
{
	if (pptt < retained->min_battery_pptt)
		retained->min_battery_pptt = pptt;

	if (pptt <= retained->max_battery_pptt - 300) // valid pptt
	{
		if (pptt <= retained->battery_pptt_saved - 500) // new interval
		{
			LOG_INF("New interval: %.2f%%", (double)pptt / 100.0);
			if (pptt <= retained->max_battery_pptt - 800) // valid interval
			{
				LOG_INF("Update interval: %.2f%%", (double)pptt / 100.0);
				update_interval(pptt);
			}
			retained->battery_runtime_saved = retained->battery_runtime_sum;
			retained->battery_pptt_saved -= 500;
		}
	}
}

static void update_curve(void)
{
	uint64_t* intervals = (uint64_t*)k_malloc(sizeof(uint64_t) * 19);
	uint64_t curve_runtime = 0;

	int8_t first_valid = -1;
	int8_t last_valid = -1;
	uint8_t valid_intervals = 0;

	// read intervals
	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		if (interval.cycles > 0)
		{
			uint64_t interval_runtime = interval.runtime / interval.cycles;
			intervals[i] = interval_runtime;
			curve_runtime += interval_runtime;
			if (first_valid < 0)
				first_valid = i;
			last_valid = i;
			valid_intervals++;
			// the valid intervals should be continuous, if there is a gap the average is used instead
		}
		else
		{
			intervals[i] = 0;
		}
	}

	if (valid_intervals < 2 || first_valid < 0) // not enough data
	{
		LOG_WRN("Not enough data to calculate discharge curve");
		k_free(intervals);
		return;
	}

	int16_t* curve = (int16_t*)k_malloc(sizeof(int16_t) * 18);
	memset(curve, 0, sizeof(int16_t) * 18);
	uint64_t runtime = 0;

	// curve can be calculated for intervals between first_valid and last_valid
	int16_t curve_start = first_valid * 500;
	int16_t curve_size = (last_valid + 1 - first_valid) * 500;

	// add missing intervals
	uint64_t average_runtime = curve_runtime / valid_intervals;
	curve_runtime += average_runtime * (last_valid + 1 - first_valid - valid_intervals);

	// calculate correct pptt for each interval from its proportional runtime
	for (uint8_t i = first_valid; i < last_valid; i++)
	{
		runtime += intervals[i] ? intervals[i] : average_runtime;
		curve[i] = runtime * curve_size / curve_runtime + curve_start;
#if DEBUG
		LOG_DBG("Map %5.2f%% -> %5.2f%%, %llu us", (i + 1) * 5.0, (double)curve[i] / 100.0, k_ticks_to_us_floor64(intervals[i]));
#endif
	}
	k_free(intervals);

	sys_write(BATT_STATS_CURVE_ID, retained->battery_pptt_curve, curve, sizeof(int16_t) * 18);
	k_free(curve);
}

static int16_t apply_curve(int16_t pptt)
{
	uint8_t interval_id = pptt / 500; // above point
	int16_t pb = (interval_id > 0 && interval_id < 19) ? retained->battery_pptt_curve[interval_id - 1] : 0;
	pb = pb ? pb : interval_id * 500;
	int16_t pa = (interval_id < 18) ? retained->battery_pptt_curve[interval_id] : 0;
	pa = pa ? pa : (interval_id + 1) * 500;
	if (pb < 0 || pa < 0 || pb > 10000 || pa > 10500 || pa < pb)
	{
		LOG_ERR("Invalid curve");
		return pptt;
	}
	return (int32_t)(pptt % 500) * (pa - pb) / 500 + pb; // linear interpolation
}

static int last_mV = -1;
static int last_unplugged_mV = -1;
static int16_t last_unplugged_pptt = -1;
static uint64_t last_unplugged_time = 0;
static uint64_t last_unplugged_runtime = 0;

static int16_t last_saved_pptt = -1;
static uint64_t last_saved_time = 0;

void sys_update_battery_tracker_voltage(int mV, bool plugged)
{
	last_mV = mV;
	if (!plugged)
	{
		last_unplugged_mV = mV;
		last_unplugged_time = k_uptime_ticks();
	}
}

void sys_update_battery_tracker(int16_t pptt, bool plugged)
{
	if (plugged)
	{
		last_saved_pptt = -1; // reset saved pptt
	}
	else
	{
		last_unplugged_pptt = pptt;
		last_unplugged_time = k_uptime_ticks();
		last_unplugged_runtime = retained->battery_runtime_sum;
		if (last_saved_pptt == -1)
		{
			last_saved_pptt = pptt;
			last_saved_time = k_uptime_ticks();
		}
	}

	if (plugged && retained->min_battery_pptt >= 0) // reset tracker while plugged
	{
		LOG_INF("Tracker reset");
		update_statistics();
		reset_tracker(-1);
		update_curve(); // recalculate curve for next discharge
	}
	else if (!plugged && retained->min_battery_pptt < 0) // unplugged, reinitialize tracker
	{
		LOG_INF("Tracker initialized: %6.2f%%", (double)pptt / 100.0);
		reset_tracker(pptt);
	}
	if (!plugged && pptt >= 0)
	{
		if (pptt < retained->min_battery_pptt - 100) // discharge (caused by long shutdown) event
		{
			LOG_WRN("Unaccounted change to battery SOC: %5.2f%% (min) -> %5.2f%% ", (double)retained->min_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
		}
		else if (pptt > retained->min_battery_pptt + 100) // possible charge (should not happen!) event
		{
			LOG_ERR("Abnormal change to battery SOC: %5.2f%% (min) -> %5.2f%% ", (double)retained->min_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
			update_curve(); // it is also possible for a device to have no usable charge indicators
		}
		else if (pptt > retained->max_battery_pptt + 100) // charge (should not happen!) event // TODO: what is a good threshold
		{
			LOG_ERR("Abnormal change to battery SOC: %5.2f%% (max) -> %6.2f%% ", (double)retained->max_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
		}
		else if (last_saved_pptt != -1 && pptt < last_saved_pptt - 100 && k_uptime_ticks() - last_saved_time <= CONFIG_SYS_CLOCK_TICKS_PER_SEC * 60) // rapid "discharge" (ex. after unplugging)
		{
			uint64_t now = k_uptime_ticks();
			uint64_t delta = k_ticks_to_us_floor64(now - last_saved_time);
			LOG_INF("Abnormal change to battery SOC: %5.2f%%/min (%5.2f%% -> %5.2f%% in %llu us)", (double)(pptt - last_saved_pptt) / 100.0 / ((double)delta / 60000000), (double)last_saved_pptt / 100.0, (double)pptt / 100.0, delta);
			update_statistics();
			reset_tracker(pptt);
			last_saved_pptt = pptt; // reset saved pptt
			last_saved_time = now;
		}
		else
		{
			update_tracker(pptt);
			update_runtime();
		}
	}

	retained_update();
}

static int16_t last_pptt = -1;
static int16_t last_calibrated_battery_pptt = -1;

int16_t sys_get_calibrated_battery_pptt(int16_t pptt)
{
	if (pptt == last_pptt)
		return last_calibrated_battery_pptt;
	last_pptt = pptt;
	last_calibrated_battery_pptt = apply_curve(pptt);
	return last_calibrated_battery_pptt;
}

int sys_get_battery_mV(void)
{
	return last_mV;
}

int sys_get_valid_battery_mV(void)
{
	if (last_unplugged_mV > 1500 && last_unplugged_mV <= 6000)
		return last_unplugged_mV;
	return -1;
}

int16_t sys_get_valid_battery_pptt(void)
{
	return last_unplugged_pptt;
}

uint64_t sys_get_last_unplugged_time(void)
{
	return last_unplugged_time;
}

uint64_t sys_get_battery_runtime_estimate(void)
{
	uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		if (interval.cycles > 0)
		{
			runtime += interval.runtime / interval.cycles;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	return runtime;
}

uint64_t sys_get_battery_runtime_min_estimate(void)
{
	uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u min: %llu us", i, k_ticks_to_us_floor64(interval.runtime_min));
#endif
		if (interval.cycles > 0)
		{
			runtime += interval.runtime_min;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime min %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	return runtime;
}

uint64_t sys_get_battery_runtime_max_estimate(void)
{
	uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u max: %llu us", i, k_ticks_to_us_floor64(interval.runtime_max));
#endif
		if (interval.cycles > 0)
		{
			runtime += interval.runtime_max;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime max %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	return runtime;
}

uint64_t sys_get_battery_remaining_time_estimate(void)
{
	if (last_unplugged_runtime <= CONFIG_SYS_CLOCK_TICKS_PER_SEC * 60) // pptt may not be valid yet
		return 0; // no valid pptt

	uint64_t runtime = sys_get_battery_runtime_estimate();
	if (runtime == 0)
		return 0; // no valid runtime

	int16_t pptt = sys_get_valid_battery_pptt();
	if (pptt < 0)
		return 0; // no valid pptt

	pptt = sys_get_calibrated_battery_pptt(pptt);
	if (pptt < 0)
		return 0;

	return runtime * pptt / 10000;
}

float sys_get_battery_cycles(void)
{
	uint32_t cycles = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		cycles += interval.cycles;
	}

	return cycles / 20.0f;
}

float sys_get_battery_calibration_coverage(void)
{
	uint8_t valid_intervals = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
		if (interval.cycles > 0)
			valid_intervals++;
	}

	return valid_intervals * (100 / 20) / 100.0f; // maximum coverage is 95%
}

int16_t sys_get_calibrated_battery_range_min_pptt(void)
{
	for (uint8_t i = 0; i < 18; i++)
	{
		if (retained->battery_pptt_curve[i] > 0)
			return i * 500;
	}
	return -1;
}

int16_t sys_get_calibrated_battery_range_max_pptt(void)
{
	for (uint8_t i = 17; i >= 0; i--)
	{
		if (retained->battery_pptt_curve[i] > 0)
			return (i + 2) * 500;
	}
	return -1;
}

int16_t sys_get_last_cycle_min_pptt(void)
{
	struct battery_tracker tracker;
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_min_battery_pptt < 0)
		return -1;
	return tracker.last_min_battery_pptt;
}

int16_t sys_get_last_cycle_max_pptt(void)
{
	struct battery_tracker tracker;
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_max_battery_pptt < 0)
		return -1;
	return tracker.last_max_battery_pptt;
}

uint64_t sys_get_last_cycle_runtime(void)
{
	struct battery_tracker tracker;
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_battery_runtime < 0)
		return 0;
	return tracker.last_battery_runtime;
}

void sys_reset_battery_tracker(void)
{
	static bool reset_confirm = false;
	if (!reset_confirm)
	{
		printk("Resetting battery tracker will clear all battery calibration data. Are you sure?\n");
		reset_confirm = true;
		return;
	}
	printk("Resetting battery tracker\n");

	reset_tracker(-1);
	struct battery_tracker tracker = {0};
	sys_write(BATT_STATS_LAST_RUN_ID, NULL, &tracker, sizeof(tracker));
	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_write(BATT_STATS_INTERVAL_0 + i, NULL, &interval, sizeof(interval));
	}
	int16_t* curve = (int16_t*)k_malloc(sizeof(int16_t) * 18);
	memset(curve, 0, sizeof(int16_t) * 18);
	sys_write(BATT_STATS_CURVE_ID, retained->battery_pptt_curve, curve, sizeof(int16_t) * 18); // updates retained
	k_free(curve);
	reset_confirm = false;
	LOG_INF("Battery tracker reset");
}