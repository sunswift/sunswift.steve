/* Intermicro communications - slave */

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/utils.h>
#include <scandal/wdt.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/ssp.h>
#include <arch/uart.h>
#include <arch/timer.h>

#include <project/mlink_slave.h>


//Helper Macros
#define bit_get(p,m) ((p) & (m)) 
#define bit_set(p,m) ((p) |= (m)) 
#define bit_clear(p,m) ((p) &= ~(m)) 
#define bit_flip(p,m) ((p) ^= (m)) 
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m)) 
#define BIT(x) (0x01 << (x))

extern uint32_t precharge_command;
extern uint32_t left_ind_command;
extern uint32_t right_ind_command;
extern uint32_t horn_state;
extern uint32_t brake_light_state;

extern uint32_t ws_right_heatsink_temp;
extern uint32_t ws_left_heatsink_temp;

extern uint32_t current_velocity;

extern uint32_t sys_state;
extern uint32_t cruise_velocity;
extern uint32_t cruise_proportional;
extern uint32_t cruise_integral;

extern ws_reduced_t Left_WS_Reduced;
extern ws_reduced_t Right_WS_Reduced;

extern sc_time_t indicator_timer;
extern sc_time_t horn_timer;
extern sc_time_t brake_timer;

extern void indicator_loop(void);
extern void horn_loop(void);
extern void brake_loop(void);

void mlink_handler() {

	uint8_t 	control;
	uint8_t		address;
	uint32_t 	*data_uint32;
	float		data_float;
	uint8_t		buffer[4];
	uint32_t	*data;
	
	//Clear Interrupt
	GPIO_IntClear(MISC0_PORT, MISC0_BIT);

	//Flush RX FIFO and begin sequence
	SSP_RX_Flush(MLINK_PORT);
	//UART_printf("mlink_sync\n\r");
	
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 0);
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 0);
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 0);
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 0);
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 1);

	//UART_printf("mlink_sync1\n\r");
	//Receive first byte - the control byte.
	SSP_Receive(MLINK_PORT, &control, 1);
	//UART_printf("mlink_sync2\n\r");
	//Receive second byte - the address byte.
	SSP_Receive(MLINK_PORT, &address, 1);
	//UART_printf("mlink_sync3\n\r");
	//Receive data
	SSP_Receive(MLINK_PORT, buffer, 4);
	//UART_printf("mlink_sync4\n\r");
	
	//Unsigned Integer 32-bits
	if((!bit_get(control, BIT(1))) && (!(bit_get(control, BIT(0))))) {
		data_uint32 = buffer;
		//UART_printf("link_handler: control: %d, addr: %d, data_int: %d\n\r", (int)control, (int)address, (int)*data_uint32);
		if(address == MLINK_PRECHARGE_ADDR) {
			precharge_command = *data_uint32;
		}
		if(address == MLINK_LEFT_IND_ADDR) {
			left_ind_command = *data_uint32;
			indicator_loop();
			indicator_timer = sc_get_timer();
		}
		if(address == MLINK_RIGHT_IND_ADDR) {
			right_ind_command = *data_uint32;
			indicator_loop();
			indicator_timer = sc_get_timer();
		}
		if(address == MLINK_HORN_ADDR) {
			horn_state = *data_uint32;
			horn_loop();
			horn_timer = sc_get_timer();
		}
		if(address == MLINK_BRAKE_LIGHT_ADDR) {
			brake_light_state = *data_uint32;
			brake_loop();
			brake_timer = sc_get_timer();
		}
		if(address == MLINK_CURRENT_VELOCITY) {
			current_velocity = *data_uint32;
		}
		if(address == MLINK_SYS_STATE) {
			sys_state = *data_uint32;
		}
		if(address == MLINK_CRUISE_VELOCITY) {
			cruise_velocity = *data_uint32;
		}
		if(address == MLINK_CRUISE_PROPORTIONAL) {
			cruise_proportional = *data_uint32;
		}
		if(address == MLINK_CRUISE_INTEGRAL) {
			cruise_integral = *data_uint32;
		}	
		if(address >= MLINK_LHS_MOTOR_OFFSET && 
			(address < (MLINK_LHS_MOTOR_OFFSET + WS_REDUCED_NUM_MEMBERS))) {
			data = &(Left_WS_Reduced.ws_type) + (address - MLINK_LHS_MOTOR_OFFSET);
			*data = *data_uint32;
			//UART_printf("link_handler: control: %d, addr: %d, data_int: %x\n\r", (int)control, (int)address, (int)*data_uint32);
		}
		if(address >= MLINK_RHS_MOTOR_OFFSET &&
			(address < (MLINK_RHS_MOTOR_OFFSET + WS_REDUCED_NUM_MEMBERS))) {
			data = &(Right_WS_Reduced.ws_type) + (address - MLINK_RHS_MOTOR_OFFSET);
			*data = *data_uint32;
		}
		//UART_printf("link_handler: control: %d, addr: %d, data_int: %x\n\r", (int)control, (int)address, (int)*data_uint32);
	}

}

void mlink_setup(void) {

	/* Initialise SSP */
	SSP_init_struct SSP_Config;	
	SSP_Config.DataSize=SSPCR0_DSS_8BIT;
	SSP_Config.FrameFormat=SSPCR0_FRF_SPI; //SPI Format
    SSP_Config.ClockPolarity=0;
    SSP_Config.ClockPhase=0;
    SSP_Config.ClockRate=MLINK_CLK_DIV; //Sets the Clock frequency of bus.
    SSP_Config.Slave=1;
    SSP_Config.ClockPrescale=2; //Greater or equal to 2, I must be

#if MLINK_PORT == 0
	SSP_IOConfig(0);
	SSP_new_Init(&SSP_Config, LPC_SSP0);
#else
	SSP_IOConfig(1); //Configure pins for SSP1
	SSP_new_Init(&SSP_Config, LPC_SSP1);
#endif
	
	/* Initialise IO Lines */
	//Set START_SYNC line to input (because we are the slave). ?? Might also need to configure pullups on this line.
	GPIO_SetDir(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT, 0);
	
    GPIO_RegisterInterruptHandler(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_FALLING,
        &mlink_handler);	
		
	//Set ACK line to output
	GPIO_SetDir(MLINK_ACK_PORT, MLINK_ACK_BIT, 1);
	GPIO_SetValue(MLINK_ACK_PORT, MLINK_ACK_BIT, 1);

}