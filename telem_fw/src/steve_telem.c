/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: The template main file. It provides a simple example of using
                 some standard scandal functions such as the UART library, the
                 CAN library, timers, LEDs, GPIOs.
                 It is designed to compile and work for the 3 micros we use on
                 the car currently, the MSP430F149 and MCP2515 combination and
                 the LPC11C14 and LPC1768 which have built in CAN controllers.

                 UART_printf is provided by the Scandal stdio library and 
                 should work well on all 3 micros.

                 If you are using this as the base for a new project, you
                 should first change the name of this file to the name of
                 your project, and then in the top level makefile, you should
                 change the CHIP and MAIN_NAME variables to correspond to
                 your hardware.

                 Don't be scared of Scandal. Dig into the code and try to
                 understand what's going on. If you think of an improvement
                 to any of the functions, then go ahead and implement it.
                 However, before committing the changes to the Scandal repo,
                 you should discuss with someone else to ensure that what 
                 you've done is a good thing ;-)

                 Keep in mind that this code is live to the public on
                 Google Code. No profanity in comments please!

    Copyright (C) Etienne Le Sueur, 2011

    Created: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Template project
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef 	DOUBLE_BUFFER_EXAMPLE
#undef 	IN_CHANNEL_EXAMPLE
#undef WAVESCULPTOR_EXAMPLE

#define INDICATOR_LOOP_MS			500
#define HORN_LOOP_MS				200
#define BRAKE_LOOP_MS				200
#define WS_SENDOUT_MS				250

#define ON							1
#define OFF							0

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>
#include <scandal/wavesculptor.h>

#include <string.h>

#if defined(lpc11c14) || defined(lpc1768)
#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#include <arch/ssp.h>
#endif // lpc11c14 || lpc1768

#include <project/mlink_slave.h>

uint32_t precharge_command;
uint32_t left_ind_command;
//uint32_t left_ind_state;
uint32_t right_ind_command;
//uint32_t right_ind_state;
uint32_t ind_state;
uint32_t horn_state;
uint32_t brake_light_state;

uint32_t current_velocity;
uint32_t sys_state;
uint32_t cruise_velocity;
uint32_t cruise_proportional;
uint32_t cruise_integral;

uint32_t ws_right_heatsink_temp;
uint32_t ws_left_heatsink_temp;

ws_reduced_t Left_WS_Reduced;
ws_reduced_t Right_WS_Reduced;

sc_time_t one_sec_timer;
sc_time_t indicator_timer;
sc_time_t horn_timer;
sc_time_t brake_timer;
sc_time_t ws_sendout_timer1;
sc_time_t ws_sendout_timer2;
sc_time_t ws_sendout_timer3;
sc_time_t ws_sendout_timer4;

uint32_t ws_sendout_state;

void indicator_loop(void);
void horn_loop(void);
void ws_sendout_loop1(void);
void ws_sendout_loop2(void);
void ws_sendout_loop3(void);
void ws_sendout_loop4(void);
void ws_sendout_loop5(void);
void brake_loop(void);

/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */
void setup(void) {

} // setup


int main(void) {

	uint8_t buffer[5] = {0xDE,0xAD,0x55,0x55,0x55};
	int i;
	uint32_t rx_buffer[2] = {0};
	setup();
	
	ind_state = OFF;
	horn_state = OFF;
	brake_light_state = OFF;

	/* Initialise the watchdog timer. If the node crashes, it will restart automatically */
	WDT_Init(); 

	/* Initialise Scandal, registers for the config messages, timesync messages and in channels */
	scandal_init();

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);

	/* Initialise UART0 */
	UART_Init(115200);

	/* Wait until UART is ready */
	scandal_delay(100);

	/* Display welcome header over UART */
	UART_printf("Welcome to the template project! This is coming out over UART0\n\r");
	UART_printf("A debug LED should blink at a rate of 1HZ\n\r");
	
	GPIO_Init();

	one_sec_timer = sc_get_timer();
	indicator_timer = sc_get_timer();
	horn_timer = sc_get_timer();
	brake_timer = sc_get_timer();
	ws_sendout_timer1 = sc_get_timer();
	ws_sendout_timer2 = sc_get_timer();
	ws_sendout_timer3 = sc_get_timer();
	ws_sendout_timer4 = sc_get_timer();
	
	mlink_setup();

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();
		
		//Handle Precharge-discharge
		if(precharge_command == MLINK_PRECHARGE_CHARGE) {
			UART_printf("PRECHARGE \n\r");
			/*Priority, Channel Number, Value*/
			scandal_send_channel(TELEM_LOW, STEVE_PRECHARGE, 1);
			precharge_command = 0x00;
		} else if(precharge_command == MLINK_PRECHARGE_DISCHARGE) {
			UART_printf("DISCHARGE \n\r");
			/*Priority, Channel Number, Value*/
			scandal_send_channel(TELEM_LOW, STEVE_PRECHARGE, 0);
			precharge_command = 0x00;
		}

		/* INDICATOR LOOP */
		if(sc_get_timer() >= indicator_timer + INDICATOR_LOOP_MS) {
			indicator_loop();
			indicator_timer = sc_get_timer();
		}
		
		/* HORN LOOP */
		if(sc_get_timer() >= horn_timer + HORN_LOOP_MS) {
			horn_loop();
			horn_timer = sc_get_timer();
		}
		
		/* BRAKE LOOP */
		if(sc_get_timer() >= brake_timer + BRAKE_LOOP_MS) {
			brake_loop();
			brake_timer = sc_get_timer();
		}
		
		/* WS Reduced Sendout Loop */
		if(sc_get_timer() >= ws_sendout_timer1 + WS_SENDOUT_MS) {
			if(ws_sendout_state == 0) {
				ws_sendout_loop1();
				ws_sendout_state = 1;
			} else if(ws_sendout_state == 1) {
				ws_sendout_loop2();
				ws_sendout_state = 2;
			} else if(ws_sendout_state == 2) {
				ws_sendout_loop3();
				ws_sendout_state = 3;
			} else if(ws_sendout_state == 3) {
				ws_sendout_loop4();
				ws_sendout_state = 4;
			} else {
				ws_sendout_loop5();
				ws_sendout_state = 0;			
			}
			ws_sendout_timer1 = sc_get_timer();
		}

		
		/* Send a UART and CAN message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			
			/*
			UART_printf("L ws_type: %d\n\r",Left_WS_Reduced.ws_type);
			UART_printf("L heatsink_temp: %d\n\r",Left_WS_Reduced.amp_hours);

			UART_printf("R ws_type: %d\n\r",Right_WS_Reduced.ws_type);
			UART_printf("R heatsink_temp: %d\n\r",Right_WS_Reduced.amp_hours);			
			*/
			UART_printf("1 second timer %u\n\r", (unsigned int)sc_get_timer());
			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}

	}
}

void indicator_loop(void) {
	uint32_t toggled = 0;
	//LEFT Indicators
	if(left_ind_command == MLINK_LEFT_IND_ON) {
		//Toggle indicators
		if(ind_state == 0) {
			toggled = 1;
			scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0xAA);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0xAA);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0xAA);
			UART_printf("LEFT_IND_ON\n\r");
			//ind_state = 1;
		} else {
			toggled = 1;
			scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
			UART_printf("LEFT_IND_OFF\n\r");
			//ind_state = 0;
		}
	} else {
		scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
		//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
		//scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_IND, 0x00);
		UART_printf("LEFT_IND_STEADY_OFF\n\r");
		//ind_state = 0;
	}
	//RIGHT Indicators
	if(right_ind_command == MLINK_RIGHT_IND_ON) {
		//Toggle indicators
		if(ind_state == 0) {
			toggled = 1;
			scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0xAA);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0xAA);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0xAA);
			UART_printf("RIGHT_IND_ON\n\r");
			//ind_state = 1;
		} else {
			toggled = 1;
			scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
			//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
			UART_printf("RIGHT_IND_OFF\n\r");
			//ind_state = 0;
		}
	} else {
		scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
		//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
		//scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_IND, 0x00);
		//ind_state = 0;
	}
	
	//if(toggled == 1) {
		if(ind_state == 0) {
			ind_state = 1;
		} else {
			ind_state = 0;
		}
	//}
}

void horn_loop(void) {
	if(horn_state == ON) {
		scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0xAA);
		//scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0xAA);
		//scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0xAA);
		UART_printf("HORN_ON\n\r");
	} else {
		scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0x00);
		//scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0x00);
		//scandal_send_channel(CONTROL_HIGH, STEVE_HORN, 0x00);
		//UART_printf("HORN_OFF\n\r");		
	}
}

void brake_loop(void) {
	if(brake_light_state == ON) {
		scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0xAA);
		//scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0xAA);
		//scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0xAA);
		UART_printf("BRAKE_ON\n\r");
	} else {
		scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0x00);
		//scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0x00);
		//scandal_send_channel(CONTROL_HIGH, STEVE_BRAKE, 0x00);
	}
}

void ws_sendout_loop1(void) {
	//Current Velocity addition
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_CURRENT_VELOCITY, current_velocity);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_CURRENT_VELOCITY, current_velocity);

	//WS TYPE
	scandal_send_channel(TELEM_LOW, STEVE_L_WS_TYPE, Left_WS_Reduced.ws_type);
	scandal_send_channel(TELEM_LOW, STEVE_R_WS_TYPE, Right_WS_Reduced.ws_type);

	//ACTIVE MOTOR
	scandal_send_channel(TELEM_LOW, STEVE_L_ACTIVE_MOTOR, Left_WS_Reduced.active_motor);
	scandal_send_channel(TELEM_LOW, STEVE_R_ACTIVE_MOTOR, Right_WS_Reduced.active_motor);
	
	UART_printf("LOOP1\n\r");

}

void ws_sendout_loop2(void) {
	//ERRORS
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_ERRORS, Left_WS_Reduced.errors);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_ERRORS, Right_WS_Reduced.errors);
	
	//LIMIT
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_LIMIT, Left_WS_Reduced.limit);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_LIMIT, Right_WS_Reduced.limit);

	//BUS CURRENT
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_BUS_CURR, Left_WS_Reduced.bus_current);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_BUS_CURR, Right_WS_Reduced.bus_current);

	UART_printf("LOOP2\n\r");
}

void ws_sendout_loop3(void) {
	//BUS VOLTAGE
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_BUS_VOLT, Left_WS_Reduced.bus_voltage);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_BUS_VOLT, Right_WS_Reduced.bus_voltage);
	
	//MOTOR VELOCITY
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_MOTOR_VEL, Left_WS_Reduced.motor_velocity);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_MOTOR_VEL, Right_WS_Reduced.motor_velocity);
	
	//PHASE_A
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_PHASE_A, Left_WS_Reduced.phase_a);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_PHASE_A, Right_WS_Reduced.phase_a);
	
	UART_printf("LOOP3\n\r");
}

void ws_sendout_loop4(void) {
	//PHASE_B
	scandal_send_channel(TELEM_LOW, STEVE_L_PHASE_B, Left_WS_Reduced.phase_b);
	scandal_send_channel(TELEM_LOW, STEVE_R_PHASE_B, Right_WS_Reduced.phase_b);

	//MOTOR_TEMP
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_MOTOR_TEMP, Left_WS_Reduced.motor_temp);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_MOTOR_TEMP, Right_WS_Reduced.motor_temp);
	
	//HEATSINK_TEMP
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_L_HEATSINK_TEMP, Left_WS_Reduced.heatsink_temp);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_R_HEATSINK_TEMP, Right_WS_Reduced.heatsink_temp);
	
	
	UART_printf("L_ERR:%x\tL_LIM:%x\tL_CUR:%d\tL_VOLT:%d\tL_VEL:%d\tL_HSTEMP:%d\n\r",
					Left_WS_Reduced.errors,
					Left_WS_Reduced.limit,
					Left_WS_Reduced.bus_current,
					Left_WS_Reduced.bus_voltage,
					Left_WS_Reduced.motor_velocity,
					Left_WS_Reduced.heatsink_temp);

	UART_printf("R_ERR:%x\tR_LIM:%x\tR_CUR:%d\tR_VOLT:%d\tR_VEL:%d\tR_HSTEMP:%d\n\r",
					Right_WS_Reduced.errors,
					Right_WS_Reduced.limit,
					Right_WS_Reduced.bus_current,
					Right_WS_Reduced.bus_voltage,
					Right_WS_Reduced.motor_velocity,
					Right_WS_Reduced.heatsink_temp);
	
	//UART_printf("Temps: %d %d\n\r",(int)Left_WS_Reduced.heatsink_temp,(int)Right_WS_Reduced.heatsink_temp);
	
	UART_printf("LOOP4\n\r");
}

void ws_sendout_loop5(void) {

	scandal_send_channel(CRITICAL_PRIORITY, STEVE_SYS_STATE, sys_state);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_CRUISE_INTEGRAL, cruise_integral);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_CRUISE_VELOCITY, cruise_velocity);
	scandal_send_channel(CRITICAL_PRIORITY, STEVE_CRUISE_PROPORTIONAL, cruise_proportional);
	
}