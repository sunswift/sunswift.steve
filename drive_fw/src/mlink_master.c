/* Intermicro communications - master */

#include <scandal/uart.h>
#include <scandal/stdio.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/ssp.h>
#include <arch/uart.h>

#include <project/mlink_master.h>


//Helper Macros
#define bit_get(p,m) ((p) & (m)) 
#define bit_set(p,m) ((p) |= (m)) 
#define bit_clear(p,m) ((p) &= ~(m)) 
#define bit_flip(p,m) ((p) ^= (m)) 
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m)) 
#define BIT(x) (0x01 << (x))

void mlink_send_ws_reduced(ws_reduced_t *WS_Reduced, uint32_t address) {
	uint32_t i;
	uint32_t *data;
	
	for(i=0; i<WS_REDUCED_NUM_MEMBERS; i++) {
		data = &(WS_Reduced->ws_type) + i;
		mlink_send(0x00, address+i, (void*) data);
		//UART_printf("WS_REDUCED: %d: %x: size %d\n\r",i,*data, WS_Reduced->errors);
	}
}

void mlink_send(uint8_t control, uint8_t address, uint8_t data[4]) {

	uint32_t i;

	//Start sync
	GPIO_SetValue(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT, 0);
	
	i = 0;
	//Wait for ACK
	while(GPIO_GetValue(MLINK_ACK_PORT, MLINK_ACK_BIT) && (i < 10000)) {
		i++;
	}
	
	/*
	if(i>20) {
		UART_printf("OVER LIMIT: %d\n\r", i);
	}
	*/
	
	SSP_Send(MLINK_PORT, &control, 1);
	SSP_Send(MLINK_PORT, &address, 1);
	SSP_Send(MLINK_PORT, data, 4);	
	
	//End sync
	GPIO_SetValue(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT, 1);
}

void mlink_setup(void) {

	/* Initialise SSP */
	SSP_init_struct SSP_Config;	
	SSP_Config.DataSize=SSPCR0_DSS_8BIT;
	SSP_Config.FrameFormat=SSPCR0_FRF_SPI; //SPI Format
    SSP_Config.ClockPolarity=0;
    SSP_Config.ClockPhase=0;
    SSP_Config.ClockRate=MLINK_CLK_DIV; //Sets the Clock frequency of bus.
    SSP_Config.Slave=0;
    SSP_Config.ClockPrescale=2; //Greater or equal to 2, I must be

#if MLINK_PORT == 0
	SSP_IOConfig(0);
	SSP_new_Init(&SSP_Config, LPC_SSP0);
#else
	SSP_IOConfig(1); //Configure pins for SSP1
	SSP_new_Init(&SSP_Config, LPC_SSP1);
#endif
	
	/* Initialise IO Lines */
	GPIO_SetDir(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT, 1);
	GPIO_SetValue(MLINK_START_SYNC_PORT, MLINK_START_SYNC_BIT, 1);
	
	GPIO_SetDir(MLINK_ACK_PORT, MLINK_ACK_BIT, 0);
		
}