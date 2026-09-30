#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

/*
LED priorities (0 is highest)
0: critical status
1: user interaction
2: boot/power
3: sensor
4: pair (esb)
5: error
6: charger
7: warn
8: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_CRITICAL 0
#define SYS_LED_PRIORITY_USER 1
#define SYS_LED_PRIORITY_BOOT 2
#define SYS_LED_PRIORITY_SENSOR 3
#define SYS_LED_PRIORITY_PAIR 4
#define SYS_LED_PRIORITY_ERROR 5
#define SYS_LED_PRIORITY_CHARGER 6
#define SYS_LED_PRIORITY_WARN 7
#define SYS_LED_PRIORITY_SYSTEM 8
#define SYS_LED_PATTERN_DEPTH 8

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
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off									// Pairing | indicates waiting (pairing)
	SYS_LED_PATTERN_LONG, // 500ms on 500ms off										// Default | indicates waiting
	SYS_LED_PATTERN_FLASH, // 200ms on 200ms off									// Default | indicates readiness

	SYS_LED_PATTERN_ONESHOT_WAKE, // 100ms on										// Success | wake from timeout
	SYS_LED_PATTERN_ONESHOT_POWERON, // 100ms on 100ms off, 2 times					// Success | wake from shutdown
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 100ms on 100ms off, 3 times				// Charging

	SYS_LED_PATTERN_ONESHOT_PROGRESS, // 200ms on 200ms off, 2 times				// Success | progress indicator
	SYS_LED_PATTERN_ONESHOT_COMPLETE, // 200ms on 200ms off, 4 times				// Success

	SYS_LED_PATTERN_ON_PERSIST, // 20% duty cycle									// Success | indicates charged
	SYS_LED_PATTERN_LONG_PERSIST, // 20% duty cycle, 500ms on 500ms off				// Charging| indicates low battery
	SYS_LED_PATTERN_PULSE_PERSIST, // 5000ms pulsing								// Charging| indicates charging
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off							// Default | indicates normal operation

	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)	// Error

	SYS_LED_PATTERN_WARNING, // 500ms on 500ms off									// Charging| indicates charger disabled
	SYS_LED_PATTERN_CRITICAL, // 200ms on 200ms off									// Error   | indicates temperature warning
};

enum sys_led_color {
	SYS_LED_COLOR_DEFAULT,
	SYS_LED_COLOR_SUCCESS,
	SYS_LED_COLOR_ERROR,
	SYS_LED_COLOR_CHARGING,
	SYS_LED_COLOR_PAIRING,
};

enum __attribute__((packed)) user_led_pattern {
	U_PATTERN_OFF = 0,
	U_PATTERN_ON = 1
};

void set_led(enum sys_led_pattern led_pattern, int priority);
void led_override(enum user_led_pattern pattern, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, uint32_t timeout);

#endif