/* Car Model */
#define	WHEEL_DIAMETER			0.558

/* System State */
#define SYS_INIT		0x00
#define SYS_MANUAL		0x01
#define SYS_CRUISE		0x02

/* Functional States */
#define	DRIVE_DISABLED		0x00
#define DRIVE_ENABLED		0x01
#define PRECHARGED			0x01
#define DISCHARGED			0x00
#define LEFT_IND_OFF		0x00
#define LEFT_IND_ON			0x01
#define RIGHT_IND_OFF		0x00
#define RIGHT_IND_ON		0x01
#define HORN_ON				0x01
#define HORN_OFF			0x00
#define HAZARDS_ON			0x01
#define HAZARDS_OFF			0x00
#define REVERSE_ON			0x01
#define REVERSE_OFF			0x00
#define BRAKE_CUTOUT_ON		0x01
#define BRAKE_CUTOUT_OFF	0x00
#define TQ_VEC_CENTRE		0x00
#define TQ_VEC_RIGHT		0x01
#define TQ_VEC_LEFT			0x02
#define REGEN_ON			0x01
#define REGEN_OFF			0x00
#define LEFT_WS_ENABLE          0x01
#define LEFT_WS_DISABLE         0x00
#define RIGHT_WS_ENABLE         0x01
#define RIGHT_WS_DISABLE        0x00

#define PADDLE_MAX			1023

#define RED_LED_PORT 3		// Port for led
#define RED_LED_BIT 0		// Bit on port for led
#define YELLOW_LED_PORT 3		// Port for led
#define YELLOW_LED_BIT 1		// Bit on port for led
#define LED_TOGGLE_TICKS 100 // 100 ticks = 1 Hz flash rate
#define FAST_LED_TOGGLE_TICKS 25 // 100 ticks = 1 Hz flash rate
#define COUNT_MAX		3 // how high to count on the LED display

/* STeVe specific */
/* Enable VLED switcher */
#define VLED_EN_PORT 2
#define VLED_EN_BIT 11
/* LED Shift Register Controls */
#define LS_RCLK_PORT 0
#define LS_RCLK_BIT 2
#define LS_SER_PORT 0
#define LS_SER_BIT 9
#define LS_SRCLK_PORT 0
#define LS_SRCLK_BIT 10
/* Buttons */
#define B0_PORT 2
#define B0_BIT 6
#define B1_PORT 2
#define B1_BIT 8
#define B2_PORT 2
#define B2_BIT 7
#define B3_PORT 1
#define B3_BIT 5
#define B4_PORT 1
#define B4_BIT 8
#define B5_PORT 1
#define B5_BIT 9
#define B6_PORT 1
#define B6_BIT 11
#define B7_PORT 2
#define B7_BIT 5
#define B8_PORT 2
#define B8_BIT 10
#define B9_PORT 0
#define B9_BIT 6
#define B10_PORT 0
#define B10_BIT 8
#define B11_PORT 0
#define B11_BIT 7

/* Functional Button Map 
#define LEFT_IND_PORT				B2_PORT
#define LEFT_IND_BIT				B2_BIT
#define RIGHT_IND_PORT				B3_PORT
#define RIGHT_IND_BIT				B3_BIT
*/

/* Failed constant button array
const int button_array_test[12][2] = {
	{B0_PORT, B0_PIN},
	{B1_PORT, B1_PIN},
	{B2_PORT, B2_PIN},
	{B3_PORT, B3_PIN},
	{B4_PORT, B4_PIN},
	{B5_PORT, B5_PIN},
	{B6_PORT, B6_PIN},
	{B7_PORT, B7_PIN},
	{B8_PORT, B8_PIN},
	{B9_PORT, B9_PIN},
	{B10_PORT, B10_PIN},
	{B11_PORT, B11_PIN}
};
*/

/* MISC Lines 
#define MISC0_PORT 3
#define MISC0_BIT 2
#define MISC1_PORT 3
#define MISC1_BIT 3
*/

/* SSP lines */
#define SSP1_CS_PORT 2
#define SSP1_CS_BIT 0

/* MAX6955 Segment Display */
#define SEG_ADDR_WRITE 					0xC0
#define SEG_GLOBAL_INTENSITY 			0x02
#define SEG_SCAN_LIMIT					0x03
#define SEG_CONTROL_REGISTER			0x04
#define SEG_DIGIT_TYPE					0x0C
#define SEG_DIGIT_BASE					0x60

/* ADC Defines*/
#define LEFT_PADDLE						0
#define LEFT_PADDLE_CT					1
#define RIGHT_PADDLE					2
#define RIGHT_PADDLE_CT					3

/* Brake Cutout */
#define BRAKE_CUTOUT_PORT				2
#define BRAKE_CUTOUT_BIT				9

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
