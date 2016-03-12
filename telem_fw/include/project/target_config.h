#define RED_LED_PORT 0		// Port for led
#define RED_LED_BIT 7		// Bit on port for led
#define YELLOW_LED_PORT 2		// Port for led
#define YELLOW_LED_BIT 7		// Bit on port for led
#define LED_TOGGLE_TICKS 100 // 100 ticks = 1 Hz flash rate
#define FAST_LED_TOGGLE_TICKS 25 // 100 ticks = 1 Hz flash rate
#define COUNT_MAX		3 // how high to count on the LED display

#define MISC0_PORT 3
#define MISC0_BIT 2
#define MISC1_PORT 3
#define MISC1_BIT 3

#define CS_PORT 2
#define CS_BIT 0

/* Wavesculptor Reduced*/
#define WS_REDUCED_NUM_MEMBERS			13
typedef struct _ws_reduced_t {
	uint32_t	ws_type;
	uint32_t	active_motor;
	uint32_t	errors;
	uint32_t	limit;
	uint32_t	bus_current;
	uint32_t	bus_voltage;
	uint32_t	motor_velocity;
	uint32_t	phase_a;
	uint32_t	phase_b;
	uint32_t	motor_temp;
	uint32_t	heatsink_temp;
	uint32_t	amp_hours;
	uint32_t	odometer;
} ws_reduced_t;