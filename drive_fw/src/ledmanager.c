//LED Manager

#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/adc.h>
#include <arch/ssp.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/uart.h>

#include <project/ledmanager.h>

uint8_t k5v_genbuf(K5V_t *K5V, uint8_t led) {
	uint8_t retbuf;
	if(led == 0) {
		retbuf = K5V->colour;
	} else {
		if(K5V->state)
			retbuf = !(K5V->colour);
		else
			retbuf = K5V->colour;
	}
	return retbuf;
}

//void manual_clockout_leds(uint8

void clockout_leds(uint32_t update_flash, steveLEDs_t *steveLEDs) {

	uint8_t ssp_buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint32_t i;
	
	//Update flashing -- TO DO - flip states if flashing flag is on.
	for(i=0;i<12;i++) {
		if(steveLEDs->K5V[i]->flash == ON) {
			steveLEDs->K5V[i]->state = !steveLEDs->K5V[i]->state;
		}
	}

	for(i=0;i<8;i++) {
		if(steveLEDs->led[i]->flash == ON) {
			if(steveLEDs->led[i]->state == ON) {
				steveLEDs->led[i]->state = OFF;
			} else {
				steveLEDs->led[i]->state = ON;
			}
		}
	}
	
	//Start constructing serial stream
	//IBUT 6,7,8
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[6]),0) << 1;
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[6]),1) << 2;
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[7]),0) << 3;
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[7]),1) << 4;
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[8]),0) << 5;
	ssp_buffer[4] |= k5v_genbuf((steveLEDs->K5V[8]),1) << 6;
	
	//IBUT 9,10,11
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[9]),0) << 1;
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[9]),1) << 2;
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[10]),0) << 3;
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[10]),1) << 4;
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[11]),0) << 5;
	ssp_buffer[3] |= k5v_genbuf((steveLEDs->K5V[11]),1) << 6;
	
	//IBUT 0,1,2
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[0]),0) << 1;
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[0]),1) << 2;
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[2]),0) << 3;
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[2]),1) << 4;
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[1]),0) << 5;
	ssp_buffer[2] |= k5v_genbuf((steveLEDs->K5V[1]),1) << 6;

	//Status LEDs
	ssp_buffer[1] |= (!steveLEDs->led[0]->state) << 0;
	ssp_buffer[1] |= (!steveLEDs->led[1]->state) << 1;
	ssp_buffer[1] |= (!steveLEDs->led[2]->state) << 2;
	ssp_buffer[1] |= (!steveLEDs->led[3]->state) << 3;
	ssp_buffer[1] |= (!steveLEDs->led[4]->state) << 4;
	ssp_buffer[1] |= (!steveLEDs->led[5]->state) << 5;
	ssp_buffer[1] |= (!steveLEDs->led[6]->state) << 6;
	ssp_buffer[1] |= (!steveLEDs->led[7]->state) << 7;
	
	//IBUT 0,1,2
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[3]),0) << 1;
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[3]),1) << 2;
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[4]),0) << 3;
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[4]),1) << 4;
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[5]),0) << 5;
	ssp_buffer[0] |= k5v_genbuf((steveLEDs->K5V[5]),1) << 6;
	
	//Clock into Shift Registers.
	SSP_Send(0, ssp_buffer, 5);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
	
	//UART_printf("CLKOUT: %x %x %x %x %x\n\r", ssp_buffer[4], ssp_buffer[3], ssp_buffer[2], ssp_buffer[1], ssp_buffer[0]);
}


void led_setup(void) {

	/* Setup LED Shift Register Clock*/
	GPIO_SetDir(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	
	/* Initialise SSP0 (LEDs) */
	SSP_init_struct SSP_Config;	
	SSP_Config.DataSize=SSPCR0_DSS_8BIT;
	SSP_Config.FrameFormat=SSPCR0_FRF_SPI; //SPI Format
    SSP_Config.ClockPolarity=0;
    SSP_Config.ClockPhase=0;
    SSP_Config.ClockRate=0; //Sets the Clock frequency of bus.
    SSP_Config.Slave=0;
    SSP_Config.ClockPrescale=2; //Greater or equal to 2, I must be

    SSP_IOConfig(0); //Configure pins for SSP0
    SSP_new_Init(&SSP_Config, LPC_SSP0);
	
}