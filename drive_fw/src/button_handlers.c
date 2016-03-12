// Button Interrupt Handlers

#define WS_TOGGLE_ENABLED 1

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/adc.h>
#include <arch/ssp.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/uart.h>

#include <scandal/wavesculptor.h>

#include <project/button_handlers.h>
#include <project/mlink_master.h>
#include <project/ledmanager.h>
#include <project/conversion.h>

//External State Variables
extern int sys_state;
extern int left_ind_state;
extern int right_ind_state;
extern int hazards_state;
extern int horn_state;
extern int drive_state;
extern int precharge_state;
extern int reverse_state;
extern int brake_cutout_state;
extern int torque_vectoring_state;
extern int regen_state;
extern int left_ws_state;
extern int right_ws_state;
extern int ws_toggle_state;

//External Cruise Variables
extern float cruise_rpm;
extern float current_rpm;

extern steveLEDs_t steveLEDs;

extern Wavesculptor_Output_Struct Left_WS;
extern Wavesculptor_Output_Struct Right_WS;

//CRUISE ENABLE/DISABLE
void cruise_handler(void) {
	if (sys_state == SYS_CRUISE) {
		sys_state = SYS_MANUAL;
                left_ws_state = LEFT_WS_ENABLE;
                right_ws_state = RIGHT_WS_ENABLE;
		torque_vectoring_state = TQ_VEC_CENTRE;
		UART_printf("MANUAL \n\r");
		steveLEDs.K5V[2]->state = OFF;
		steveLEDs.K5V[2]->flash = OFF;
		steveLEDs.K5V[3]->state = OFF;
		steveLEDs.K5V[3]->flash = OFF;
		steveLEDs.K5V[6]->state = OFF;
		steveLEDs.K5V[6]->flash = OFF;
		steveLEDs.K5V[11]->state = OFF;
		steveLEDs.K5V[11]->flash = OFF;
	} else if ((sys_state == SYS_MANUAL) && (reverse_state == REVERSE_OFF)) {
		sys_state = SYS_CRUISE;
                left_ws_state = LEFT_WS_ENABLE;
                right_ws_state = RIGHT_WS_ENABLE;
		cruise_rpm = current_rpm;
		UART_printf("CRUISE \n\r");
		steveLEDs.K5V[2]->state = OFF;
		steveLEDs.K5V[2]->flash = OFF;
		steveLEDs.K5V[3]->state = OFF;
		steveLEDs.K5V[3]->flash = OFF;
		steveLEDs.K5V[6]->colour = GREEN;
		steveLEDs.K5V[6]->flash = ON;
		steveLEDs.K5V[11]->colour = GREEN;
		steveLEDs.K5V[11]->flash = ON;
	}
}

//CRUISE UP - TQ VECTORING RIGHT
void cruise_up_handler(void) {
	if (sys_state == SYS_CRUISE) {
		cruise_rpm += mps2rpm(kph2mps(1), WHEEL_DIAMETER);
	} else if (sys_state == SYS_MANUAL) {
		if (torque_vectoring_state == TQ_VEC_CENTRE) {
			torque_vectoring_state = TQ_VEC_RIGHT;
		} else if (torque_vectoring_state == TQ_VEC_LEFT) {
			torque_vectoring_state = TQ_VEC_CENTRE;
		}
		//scandal_send_ws_reset(&Left_WS);
		//scandal_send_ws_reset(&Right_WS);
	}
}

//CRUISE DOWN - TQ VECTORING LEFT
void cruise_down_handler(void) {
	if (sys_state == SYS_CRUISE) {
		cruise_rpm -= mps2rpm(kph2mps(1), WHEEL_DIAMETER);
	} else if (sys_state == SYS_MANUAL) {
		if (torque_vectoring_state == TQ_VEC_CENTRE) {
			torque_vectoring_state = TQ_VEC_LEFT;
		} else if (torque_vectoring_state == TQ_VEC_RIGHT) {
			torque_vectoring_state = TQ_VEC_CENTRE;
		}
	}
}

//DRIVE ENABLE/DISABLE
void drive_enable_handler(void) {
	uint32_t data;
	if (drive_state == DRIVE_DISABLED) {
		UART_printf("DRIVE ENABLED\n\r");
		drive_state = DRIVE_ENABLED;
		precharge_state = PRECHARGED;
		torque_vectoring_state = TQ_VEC_CENTRE;
		data = MLINK_PRECHARGE_CHARGE;
		//sys_state = SYS_MANUAL;
		mlink_send(0x00, MLINK_PRECHARGE_ADDR, (void*) &data);
		//steveLEDs.led[2]->state = ON;
		//steveLEDs.led[2]->flash = OFF;
		//steveLEDs.led[5]->state = ON;
		//steveLEDs.led[5]->flash = OFF;
                left_ws_state = LEFT_WS_ENABLE;
                right_ws_state = RIGHT_WS_ENABLE;
                steveLEDs.K5V[2]->flash = OFF;
                steveLEDs.K5V[2]->state = OFF;
                steveLEDs.K5V[3]->flash = OFF;
                steveLEDs.K5V[3]->state = OFF;
	} else {
		UART_printf("DRIVE_DISABLED\n\r");
		drive_state = DRIVE_DISABLED;
		precharge_state = DISCHARGED;
		data = MLINK_PRECHARGE_DISCHARGE;
		//sys_state = SYS_INIT;
		mlink_send(0x00, MLINK_PRECHARGE_ADDR, (void*) &data);
		//steveLEDs.led[2]->state = OFF;
		//steveLEDs.led[2]->flash = OFF;
		//steveLEDs.led[5]->state = OFF;
		//steveLEDs.led[5]->flash = OFF;
	}
}

//LEFT INDICATOR - BUT2
void left_ind_handler(void) {
	uint32_t data;
	if (left_ind_state == LEFT_IND_OFF) {
		left_ind_state = LEFT_IND_ON;
		right_ind_state = RIGHT_IND_OFF;
		hazards_state = HAZARDS_OFF;
		UART_printf("LEFT_IND_ON\n\r");
		data = MLINK_LEFT_IND_ON;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		data = MLINK_RIGHT_IND_OFF;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[2]->colour = ORANGE;
		steveLEDs.K5V[2]->flash = ON;
		steveLEDs.K5V[3]->flash = OFF;
		steveLEDs.K5V[3]->state = OFF;
		steveLEDs.K5V[9]->flash = OFF;
		steveLEDs.K5V[9]->state = OFF;
	} else {
		left_ind_state = LEFT_IND_OFF;
		UART_printf("LEFT_IND_OFF\n\r");
		data = MLINK_LEFT_IND_OFF;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		data = MLINK_RIGHT_IND_OFF;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[2]->flash = OFF;
		steveLEDs.K5V[2]->state = OFF;
	}
}

//RIGHT INDICATOR - BUT3
void right_ind_handler(void) {
	uint32_t data;
	if (right_ind_state == RIGHT_IND_OFF) {
		left_ind_state = LEFT_IND_OFF;
		right_ind_state = RIGHT_IND_ON;
		hazards_state = HAZARDS_OFF;
		UART_printf("RIGHT_IND_ON\n\r");
		data = MLINK_RIGHT_IND_ON;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		data = MLINK_LEFT_IND_OFF;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[3]->colour = ORANGE;
		steveLEDs.K5V[3]->flash = ON;
		steveLEDs.K5V[2]->flash = OFF;
		steveLEDs.K5V[2]->state = OFF;
		steveLEDs.K5V[9]->flash = OFF;
		steveLEDs.K5V[9]->state = OFF;
	} else {
		right_ind_state = RIGHT_IND_OFF;
		UART_printf("RIGHT_IND_OFF\n\r");
		data = MLINK_RIGHT_IND_OFF;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		data = MLINK_LEFT_IND_OFF;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[3]->flash = OFF;
		steveLEDs.K5V[3]->state = OFF;
	}
}

//HORN - BUT8
void horn_handler(void) {
	uint32_t data;
	// If button is depressed - need this for continuous update.
	if(GPIO_GetValue(B8_PORT, B8_BIT)) {
		horn_state = HORN_ON;
		//UART_printf("HORN ON\n\r");
		data = MLINK_HORN_ON;
		mlink_send(0x00, MLINK_HORN_ADDR, (void*) &data);
		steveLEDs.K5V[8]->colour = ORANGE;
		steveLEDs.K5V[8]->flash = ON;
	} else {
		horn_state = HORN_OFF;
		//UART_printf("HORN OFF\n\r");
		data = MLINK_HORN_OFF;
		mlink_send(0x00, MLINK_HORN_ADDR, (void*) &data);
		steveLEDs.K5V[8]->flash = OFF;
		steveLEDs.K5V[8]->state = OFF;
	}
}

//HAZARDS - BUT9
void hazards_handler(void) {
	uint32_t data;
	if (hazards_state == HAZARDS_OFF) {
		left_ind_state = LEFT_IND_OFF;
		right_ind_state = RIGHT_IND_OFF;
		hazards_state = HAZARDS_ON;
		UART_printf("HAZARDS_ON\n\r");
		data = MLINK_RIGHT_IND_ON;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		data = MLINK_LEFT_IND_ON;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[9]->colour = ORANGE;
		steveLEDs.K5V[9]->flash = ON;
		steveLEDs.K5V[2]->flash = OFF;
		steveLEDs.K5V[2]->state = OFF;
		steveLEDs.K5V[3]->flash = OFF;
		steveLEDs.K5V[3]->state = OFF;
	} else {
		hazards_state = HAZARDS_OFF;
		UART_printf("HAZARDS_OFF\n\r");
		data = MLINK_RIGHT_IND_OFF;
		mlink_send(0x00, MLINK_RIGHT_IND_ADDR, (void*) &data);
		data = MLINK_LEFT_IND_OFF;
		mlink_send(0x00, MLINK_LEFT_IND_ADDR, (void*) &data);
		steveLEDs.K5V[9]->flash = OFF;
		steveLEDs.K5V[9]->state = OFF;
	}
}

//REVERSE - BUT1
void reverse_handler(void) {
	uint32_t i;
	if (reverse_state == REVERSE_OFF && sys_state == SYS_MANUAL) {
		reverse_state = REVERSE_ON;
		UART_printf("REVERSE_ON\n\r");
		/*for(i=0;i<8;i++) {
			steveLEDs.led[i]->flash = ON;
		}*/
		steveLEDs.K5V[1]->colour = ORANGE;
		steveLEDs.K5V[1]->flash = ON;
	} else {
		reverse_state = REVERSE_OFF;
		UART_printf("REVERSE_OFF\n\r");
		/*
		for(i=0;i<8;i++) {
			steveLEDs.led[i]->flash = OFF;
			steveLEDs.led[i]->state = OFF;
		}*/
		steveLEDs.K5V[1]->flash = OFF;
		steveLEDs.K5V[1]->state = OFF;
	}
}

//RESET WS - BUT4
void reset_ws_handler(void) {
	UART_printf("RESET WS\n\r");
	scandal_send_ws_reset(&Left_WS);
	scandal_send_ws_reset(&Right_WS);
}

void left_ws_enable_handler(void) {
        if (sys_state == SYS_MANUAL) {
                if (left_ws_state == LEFT_WS_ENABLE) {
                        left_ws_state = LEFT_WS_DISABLE;
                        steveLEDs.K5V[2]->colour = ORANGE;
                        steveLEDs.K5V[2]->flash = ON;
                } else {
                        left_ws_state = LEFT_WS_ENABLE;
                        steveLEDs.K5V[2]->flash = OFF;
                        steveLEDs.K5V[2]->state = OFF;
                }
        }
}

void right_ws_enable_handler(void) {
        if (sys_state == SYS_MANUAL) {
                if (right_ws_state == RIGHT_WS_ENABLE) {
                        right_ws_state = RIGHT_WS_DISABLE;
                        steveLEDs.K5V[3]->colour = ORANGE;
                        steveLEDs.K5V[3]->flash = ON;
                } else {
                        right_ws_state = RIGHT_WS_ENABLE;
                        steveLEDs.K5V[3]->flash = OFF;
                        steveLEDs.K5V[3]->state = OFF;
                }
        }
}

void but0_handler(void) {
	cruise_down_handler();
	//UART_printf("BUT0: %d\n\r", GPIO_GetValue(B0_PORT, B0_BIT));
	GPIO_IntClear(B0_PORT, B0_BIT);
}

void but1_handler(void) {
	reverse_handler();
	//UART_printf("BUT1: %d\n\r", GPIO_GetValue(B1_PORT, B1_BIT));
	GPIO_IntClear(B1_PORT, B1_BIT);
}

void but2_handler(void) {
        if (ws_toggle_state == WS_TOGGLE_ENABLED) {
                left_ws_enable_handler();
        } else {
                left_ind_handler();
        }
	//UART_printf("BUT2: %d\n\r", GPIO_GetValue(B2_PORT, B2_BIT));
	GPIO_IntClear(B2_PORT, B2_BIT);
}

void but3_handler(void) {
        if (ws_toggle_state == WS_TOGGLE_ENABLED) {
                right_ws_enable_handler();
        } else {
                right_ind_handler();
        }
	//UART_printf("BUT3: %d\n\r", GPIO_GetValue(B3_PORT, B3_BIT));
	GPIO_IntClear(B3_PORT, B3_BIT);
}

void but4_handler(void) {
	//UART_printf("BUT4: %d\n\r", GPIO_GetValue(B4_PORT, B4_BIT));
	reset_ws_handler();
	GPIO_IntClear(B4_PORT, B4_BIT);
}

void but5_handler(void) {
	cruise_up_handler();
	//UART_printf("BUT5: %d\n\r", GPIO_GetValue(B5_PORT, B5_BIT));
	GPIO_IntClear(B5_PORT, B5_BIT);
}

void but6_handler(void) {
	cruise_handler();
	//UART_printf("BUT6: %d\n\r", GPIO_GetValue(B6_PORT, B6_BIT));
	GPIO_IntClear(B6_PORT, B6_BIT);
}

void but7_handler(void) {
	drive_enable_handler();
	//UART_printf("BUT7: %d\n\r", test);
	GPIO_IntClear(B7_PORT, B7_BIT);
}

void but8_handler(void) {
	horn_handler();
	//UART_printf("BUT8: %d\n\r", GPIO_GetValue(B8_PORT, B8_BIT));
	GPIO_IntClear(B8_PORT, B8_BIT);
}

void but9_handler(void) {
	hazards_handler();
	//UART_printf("BUT9: %d\n\r", GPIO_GetValue(B9_PORT, B9_BIT));
	GPIO_IntClear(B9_PORT, B9_BIT);
}

void but10_handler(void) {
	drive_enable_handler();
	//UART_printf("BUT10: %d\n\r", GPIO_GetValue(B10_PORT, B10_BIT));
	GPIO_IntClear(B10_PORT, B10_BIT);
}

void but11_handler(void) {
	cruise_handler();
	//UART_printf("BUT11: %d\n\r", GPIO_GetValue(B11_PORT, B11_BIT));
	GPIO_IntClear(B11_PORT, B11_BIT);
}

void brake_cutout_handler(void) {
	uint32_t data;
	if(GPIO_GetValue(BRAKE_CUTOUT_PORT, BRAKE_CUTOUT_BIT) || (regen_state == REGEN_ON)) {
		brake_cutout_state = BRAKE_CUTOUT_ON;
		if(sys_state == SYS_CRUISE) {
			sys_state == SYS_MANUAL;
			steveLEDs.K5V[6]->state = OFF;
			steveLEDs.K5V[6]->flash = OFF;
			steveLEDs.K5V[11]->state = OFF;
			steveLEDs.K5V[11]->flash = OFF;
		}
		//UART_printf("BRAKE_CUTOUT_ON\n\r");
		data = MLINK_BRAKE_LIGHT_ON;
		mlink_send(0x00, MLINK_BRAKE_LIGHT_ADDR, (void*) &data);
	} else {
		brake_cutout_state = BRAKE_CUTOUT_OFF;
		//UART_printf("BRAKE_CUTOUT_OFF\n\r");
		data = MLINK_BRAKE_LIGHT_OFF;
		mlink_send(0x00, MLINK_BRAKE_LIGHT_ADDR, (void*) &data);
	}
	GPIO_IntClear(BRAKE_CUTOUT_PORT, BRAKE_CUTOUT_BIT);
}


void setup_button_interrupts(void) {
	//Brake CUTOUT
    GPIO_RegisterInterruptHandler(BRAKE_CUTOUT_PORT, BRAKE_CUTOUT_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &brake_cutout_handler);
	
	//K5V
    GPIO_RegisterInterruptHandler(B0_PORT, B0_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but0_handler);
    GPIO_RegisterInterruptHandler(B1_PORT, B1_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but1_handler);	
    GPIO_RegisterInterruptHandler(B2_PORT, B2_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but2_handler);
    GPIO_RegisterInterruptHandler(B3_PORT, B3_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but3_handler);
    GPIO_RegisterInterruptHandler(B4_PORT, B4_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but4_handler);
    GPIO_RegisterInterruptHandler(B5_PORT, B5_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but5_handler);	
    GPIO_RegisterInterruptHandler(B6_PORT, B6_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but6_handler);
    GPIO_RegisterInterruptHandler(B7_PORT, B7_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but7_handler);
    GPIO_RegisterInterruptHandler(B8_PORT, B8_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but8_handler);
    GPIO_RegisterInterruptHandler(B9_PORT, B9_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but9_handler);	
    GPIO_RegisterInterruptHandler(B10_PORT, B10_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but10_handler);
    GPIO_RegisterInterruptHandler(B11_PORT, B11_BIT,
        GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_RISING,
        &but11_handler);
}
