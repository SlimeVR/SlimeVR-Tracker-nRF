#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: connection (esb)
3: status
4: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_BOOT 0
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PRIORITY_CONNECTION 2
#define SYS_LED_PRIORITY_STATUS 3
#define SYS_LED_PRIORITY_SYSTEM 4
#define SYS_LED_PATTERN_DEPTH 5

// RGB
// Red, Green, Blue

// Tri-color
// Red/Amber, Green, YellowGreen/White

// RG
// Red, Green

// Dual color
// Red/Amber, YellowGreen/White

// TODO: these patterns are kinda funky
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF, // yield to lower priority patterns
	SYS_LED_PATTERN_ON,																// Default | indicates busy
	SYS_LED_PATTERN_SHORT, // Two rapid blue fades per second							// Pairing mode search
	SYS_LED_PATTERN_LONG, // 500ms on 500ms off										// Default | indicates waiting
	SYS_LED_PATTERN_FLASH, // 200ms on 200ms off									// Default | indicates readiness

	SYS_LED_PATTERN_ONESHOT_POWERON, // 200ms on 200ms off, 3 times					// Default
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off				// Default
	SYS_LED_PATTERN_ONESHOT_PROGRESS, // 200ms on 200ms off, 2 times				// Success
	SYS_LED_PATTERN_ONESHOT_COMPLETE, // 200ms on 200ms off, 4 times				// Pairing success (green)

	SYS_LED_PATTERN_ON_PERSIST, // 20% duty cycle									// Success | indicates charged
	SYS_LED_PATTERN_LONG_PERSIST, // 20% duty cycle, 500ms on 500ms off						// Low battery
	SYS_LED_PATTERN_PULSE_PERSIST, // 5000ms pulsing								// Charging
	SYS_LED_PATTERN_ACTIVE_PERSIST, // Quick purple fade pulse, long idle				// Connected and operating normally
	SYS_LED_PATTERN_UNCALIBRATED_ALERT, // Two quick orange flashes every 5 seconds				// Gyro bias missing

	SYS_LED_PATTERN_CALIBRATION_PROGRESS, // 12s orange→green fade with accelerating pulse		// Gyro bias calibration running
	SYS_LED_PATTERN_CALIBRATION_SUCCESS, // 4s purple confirmation sequence					// Gyro bias calibration complete


	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)	// Error
};

enum sys_led_color {
	SYS_LED_COLOR_DEFAULT,
	SYS_LED_COLOR_SUCCESS,
	SYS_LED_COLOR_ERROR,
	SYS_LED_COLOR_CHARGING,
	SYS_LED_COLOR_LOW_BATTERY,
	SYS_LED_COLOR_PAIRING,
	SYS_LED_COLOR_PAIRING_SUCCESS,
	SYS_LED_COLOR_DYNAMIC,
};

void set_led(enum sys_led_pattern led_pattern, int priority);

#endif
