//LED Manager

#define ORANGE 		1
#define GREEN 		0

#define ON			1
#define OFF			0

typedef struct _K5V_t {
	uint32_t state;
	uint32_t colour;
	uint32_t flash;
} K5V_t;

typedef struct _led_t {
	uint32_t state;
	uint32_t flash;
} led_t;

typedef struct _steveLEDs_t {
	K5V_t 		*K5V[12];
	led_t 		*led[8];
} steveLEDs_t;

void clockout_leds(uint32_t update_flash, steveLEDs_t *steveLEDs);
void led_setup(void);
