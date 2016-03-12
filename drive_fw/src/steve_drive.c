/* --------------------------------------------------------------------------                                 
    Steve DRIVE
    File name: steve_drive.c
    Author: Stephanie Ascone
    Description: Steve Drive

   -------------------------------------------------------------------------- */
/* 
 * This file is part of the project STEVE
 * 
 * This is free software: you can redistribute it and/or modify
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

/* 
 * Enalbes/disables drive cutout when the brake cutout switch is pressed
 * 0 enables drive cutout (default)
 * 1 disables drive cutout
 */
#define DRIVE_CUTOUT_OVER_RIDE 1
/*
 * Set to 1 to disable regen in cruise mode
 */
#define CRUISE_REGEN_DISABLE 1
/*
 * Set to 1 to change indicator buttons to motor controller enable/disable
 * buttons, where the left indicator button will toggle the left motor
 * controller on and off and the right indicator will do the same for the 
 * right motor controller
 */
#define WS_TOGGLE_ENABLE 1

#define	USE_AVERAGE_VELOCITY 0

#define PROPORTIONAL_GAIN 		0.01 //was 0.01
#define INTEGRAL_GAIN 			0.0001 //was 0.000002

#define BASE_YEL_LED_PORT 3
#define BASE_YEL_LED_BIT 1
#define BASE_RED_LED_PORT 3
#define BASE_RED_LED_BIT 0

#define LEDS_FLASH_MS			200
#define STATE_CONFIRM_MS		200
#define WS_SENDOUT_MS			200

#include <scandal/wavesculptor.h>
#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>
#include <scandal/error.h>

#include <string.h>

#if defined(lpc11c14) || defined(lpc1768)
#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/adc.h>
#include <arch/ssp.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#endif // lpc11c14 || lpc1768

#include <project/mlink_master.h>
#include <project/ledmanager.h>
#include <project/button_handlers.h>
#include <project/conversion.h>

#define TORQUE_MODE_REGEN   0
#define TORQUE_MODE_OPEN    1
#define TORQUE_MODE_LIMSLIP 2
#define TORQUE_MODE_VECTOR  3

#define VELOCITY_MAX			40.0
#define	VELOCITY_REVERSE_MAX	-10.0

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

/* Cruise related globals */
float cruise_velocity = 0.0;
float current_velocity = 0.0;
float current_rpm = 0.0;
float cruise_rpm = 0.0;
float cruise_integral = 0.0;
float cruise_torque = 0.0;
float cruise_proportional = 0.0;

/* Adafruit Breathing Table */
uint8_t LED_Breathe_Table[]  = {   	80,  87,  95, 103, 112, 121, 131, 141, 
									151, 161, 172, 182, 192, 202, 211, 220,
									228, 236, 242, 247, 251, 254, 255, 255, 
									254, 251, 247, 242, 236, 228, 220, 211,
									202, 192, 182, 172, 161, 151, 141, 131, 
									121, 112, 103,  95,  87,  80,  73,  66,
									60,  55,  50,  45,  41,  38,  34,  31,  
									28,  26,  24,  22,  20,  20,  20,  20,
									20,  20,  20,  20,  20,  20,  20,  20,  
									20,  20,  20,  20,  20,  20,  20,  20,
									20,  20,  20,  20,  20,  20,  20,  20,  
									20,  20,  20,  20,  22,  24,  26,  28,
									31,  34,  38,  41,  45,  50,  55,  60,  
									66,  73 };

uint32_t but0;
uint8_t fade_on;

//System States
uint32_t sys_state;

//Functional States
uint32_t precharge_state;
uint32_t drive_state;
uint32_t left_ind_state;
uint32_t right_ind_state;
uint32_t horn_state;
uint32_t hazards_state;
uint32_t reverse_state;
uint32_t brake_cutout_state;
uint32_t torque_vectoring_state;
uint32_t alphanumeric_state;
uint32_t regen_state;
uint32_t left_ws_state;
uint32_t right_ws_state;
uint32_t ws_toggle_state;

uint8_t buffer[3][5] = {
		{0xAA,0xFF,0xAA,0xAA,0xAA},
		{0x00,0xFF,0x00,0x00,0x00},
		{0x00,0xFF,0x00,0x00,0x00}};
		
uint8_t ssp_buffer[3];

//Wavesculptor Structs
Wavesculptor_Output_Struct Left_WS;
Wavesculptor_Output_Struct Right_WS;

//Wavesculptor Reduced Structs
ws_reduced_t Left_WS_Reduced;
ws_reduced_t Right_WS_Reduced;

//LED Manager
steveLEDs_t steveLEDs;
K5V_t		pK5V[12];
led_t		pled[8];

int read_all_buttons(void);

float massage_paddle(uint32_t paddle_channel);

int32_t torqueControl(uint8_t controlMode, int32_t targetRPM, float desiredTorque, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct);
float absoluteFloat(float input);
float limit(float input, float limitAmount);
float absLimit(float input, float limit);
float max(float left, float right);
float abs(float input);

//I2C Helpers (put in helper files on Friday please!) and for MAX6955
void I2CWrite_3(uint8_t addr, uint8_t reg, uint8_t data);
void seg_setup(void);
void seg_write_char(uint8_t digit, uint8_t font);
void seg_dual_int (uint8_t init_digit, uint8_t integer);
void seg_dual_float (uint8_t init_digit, float data);

float calculateTorque(void);

void std_msg_handler(can_msg *msg){
    //UART_printf("STD_RCVD %d id= %d\r\n", sc_get_timer(), msg->id-0x420);
    uint16_t baseAddress = msg->id & 0x7E0;
    
    if(baseAddress == Left_WS.BaseAddress){
            scandal_store_ws_message(msg, &Left_WS);
    }else if(baseAddress == Right_WS.BaseAddress){
            scandal_store_ws_message(msg, &Right_WS);
    }else{
        UART_printf("Daaam, got some other message I wasn't expecting :S ID = 0x%x\r\n", (unsigned int) msg->id);
    }
    //scandal_store_ws_message(msg, &Left_WS);
}

void setup(void) {

	uint32_t i;

	GPIO_Init();
	GPIO_SetDir(BASE_RED_LED_PORT, BASE_RED_LED_BIT, 1);
	GPIO_SetDir(BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, 1);

	/* Setup VLED*/
	GPIO_SetDir(VLED_EN_PORT, VLED_EN_BIT, 1);
	GPIO_SetValue(VLED_EN_PORT, VLED_EN_BIT, 1);

	/* Setup Buttons */
	GPIO_SetDir(B0_PORT, B0_BIT, 0);
	GPIO_SetDir(B1_PORT, B1_BIT, 0);
	GPIO_SetDir(B2_PORT, B2_BIT, 0);
	GPIO_SetDir(B3_PORT, B3_BIT, 0);
	GPIO_SetDir(B4_PORT, B4_BIT, 0);
	GPIO_SetDir(B5_PORT, B5_BIT, 0);
	GPIO_SetDir(B6_PORT, B6_BIT, 0);
	GPIO_SetDir(B7_PORT, B7_BIT, 0);
	GPIO_SetDir(B8_PORT, B8_BIT, 0);
	GPIO_SetDir(B9_PORT, B9_BIT, 0);
	GPIO_SetDir(B10_PORT, B10_BIT, 0);
	GPIO_SetDir(B11_PORT, B11_BIT, 0);
	
	/* Setup LED Shift Registers */
	GPIO_SetDir(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	
	/* Setup I2C */
	I2CInit((uint32_t)I2CMASTER);
	
	//Wavesculptor Address
    Left_WS.BaseAddress  = 0x400;
    Right_WS.BaseAddress = 0x420;
    
    Left_WS.ControlAddress  = 0x500;
    Right_WS.ControlAddress = 0x520;	
	
	// MLINK
	mlink_setup();
	
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
	
	/* Setup Button Interrupts */
	setup_button_interrupts();

	/* Initialise the watchdog timer. If the node crashes, it will restart automatically */
	WDT_Init(); 
	
	/* Initialise UART0 */
	UART_Init(115200);
	
	//Wavesculptor
	register_standard_message_handler(std_msg_handler);
	
	/* Initialise Scandal, registers for the config messages, timesync messages and in channels */
	scandal_init();
	
	//Charith's Hack - WS Filter messages... pull out velocity.
    can_register_id(0x7FF, 0x403, 0, 0);
    can_register_id(0x7FF, 0x423, 0, 0);
	
	//Charith's Hack.
	while(can_register_id(0, 0, 0, 0)==NO_ERR)
        ;	

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);

	/* Wait until UART is ready */
	scandal_delay(1000);

	/* Display welcome header over UART */
	UART_printf("Welcome to the template project! This is coming out over UART0\n\r");
	UART_printf("A debug LED should blink at a rate of 1HZ\n\r");

	red_led(1);

	//Setup ADC
	
	ADC_Init(0);
	ADC_EnableChannel(0);
	ADC_EnableChannel(1);
	ADC_EnableChannel(2);
	ADC_EnableChannel(3);
	
	seg_setup();
	
	//Steve LEDs
	for(i=0;i<8;i++) {
		steveLEDs.led[i] = &pled[i];
	}
	for(i=0;i<12;i++) {
		steveLEDs.K5V[i] = &pK5V[i];
	}
	
} // setup

/* Toggle LED Shift RCLK */
void trigger_led_rclk(void) {
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 1);
	GPIO_SetValue(LS_RCLK_PORT, LS_RCLK_BIT, 0);
}

uint32_t get_ADC(uint32_t channelNum) {
	uint32_t i = 0;
	ADC_Read(channelNum);
	while((!ADCIntDone) && (i<10)) {
		i++;
	}
	//UART_printf("ADC_I:%d\n\r",i);
	scandal_delay(1);
	
	if(i>=10) {
		return PADDLE_MAX;
	} else {
		return ADCValue[channelNum];
	}
}

int main(void) {

	uint32_t data;

	setup();
	
	/* STATE INIT */
	sys_state = SYS_INIT;
	drive_state = DRIVE_DISABLED;
	precharge_state = DISCHARGED;
	reverse_state = REVERSE_OFF;
	brake_cutout_state = BRAKE_CUTOUT_ON;
	
	left_ind_state = LEFT_IND_OFF;
	right_ind_state = RIGHT_IND_OFF;
	horn_state = HORN_OFF;
	hazards_state = HAZARDS_OFF;
	
        left_ws_state = LEFT_WS_ENABLE;
        right_ws_state = RIGHT_WS_ENABLE;
        ws_toggle_state = WS_TOGGLE_ENABLE;

	torque_vectoring_state = TQ_VEC_CENTRE;
	
	sc_time_t one_sec_timer = sc_get_timer();
	sc_time_t hundred_ms_timer = sc_get_timer();
	sc_time_t leds_flash_timer = sc_get_timer();
	sc_time_t state_confirm_timer = sc_get_timer();
	sc_time_t ws_sendout_timer = sc_get_timer();

	int fade_count = 0;
	int fade_max = 63;
	int fade_level = 0;
	int fade_level_max = sizeof(LED_Breathe_Table)*8;
		
	int tog_bit = 0;
		
	uint8_t buffer2[1] = {0};
		
	while (1) {

		handle_scandal();
		
		int i;
		
		if(sys_state == SYS_INIT && precharge_state == PRECHARGED) {
			sys_state = SYS_MANUAL;
			//buffer[2][4] = 0x02;
		}
		
		if(precharge_state == DISCHARGED) {
			drive_state = DRIVE_DISABLED;
			sys_state = SYS_INIT;
			steveLEDs.K5V[6]->state = OFF;
			steveLEDs.K5V[6]->flash = OFF;
			steveLEDs.K5V[11]->state = OFF;
			steveLEDs.K5V[11]->flash = OFF;
		}
		
		if((sys_state == SYS_CRUISE) && (brake_cutout_state == BRAKE_CUTOUT_ON)) {
			sys_state = SYS_MANUAL;
		}

		if(sys_state == SYS_INIT) {
			if(fade_count > LED_Breathe_Table[fade_level/8]/4) {
				//Turn on
				if(fade_on != 1) {
					SSP_Send(0, buffer[0], 5);
					trigger_led_rclk();
					fade_on = 1;
				}
			} else {
				//Turn off
				if(fade_on == 1) {
					SSP_Send(0, buffer[1], 5);
					trigger_led_rclk();
					fade_on = 0;
				}
			}
			if(fade_count <= fade_max) {
				fade_count++;
			} else {
				fade_count = 0;
				if(fade_level <= fade_level_max) {
					fade_level++;
				} else {
					fade_level = 0;
				}
			}
		}
		
		//WS Sendout Loop
		if((sys_state != SYS_INIT) && sc_get_timer() >= ws_sendout_timer + WS_SENDOUT_MS) {
		
			//Construct
			Left_WS_Reduced.ws_type = Left_WS.TritiumID;
			Left_WS_Reduced.active_motor = Left_WS.ActiveMotor;
			Left_WS_Reduced.errors = Left_WS.ErrorFlags;
			Left_WS_Reduced.limit = Left_WS.LimitFlags;
			Left_WS_Reduced.bus_current = (int)(Left_WS.BusCurrent*1000.0);
			Left_WS_Reduced.bus_voltage = (int)(Left_WS.BusVoltage*1000.0);
			Left_WS_Reduced.motor_velocity = (int)(Left_WS.MotorVelocity*1000.0);
			Left_WS_Reduced.phase_a = (int)(Left_WS.Phase_1*1000.0);
			Left_WS_Reduced.phase_b = (int)(Left_WS.Phase_2*1000.0);
			Left_WS_Reduced.motor_temp = (int)(Left_WS.MotorTemp*1000.0);
			Left_WS_Reduced.heatsink_temp = (int)(Left_WS.HeatsinkTemp*1000.0);
			Left_WS_Reduced.amp_hours = (int)(Left_WS.DCBusAHr*1000.0);	
			Left_WS_Reduced.odometer = (int)(Left_WS.Odometer*1000.0);

			Right_WS_Reduced.ws_type = Right_WS.TritiumID;
			Right_WS_Reduced.active_motor = Right_WS.ActiveMotor;
			Right_WS_Reduced.errors = Right_WS.ErrorFlags;
			Right_WS_Reduced.limit = Right_WS.LimitFlags;
			Right_WS_Reduced.bus_current = (int)(Right_WS.BusCurrent*1000.0);
			Right_WS_Reduced.bus_voltage = (int)(Right_WS.BusVoltage*1000.0);
			Right_WS_Reduced.motor_velocity = (int)(Right_WS.MotorVelocity*1000.0);
			Right_WS_Reduced.phase_a = (int)(Right_WS.Phase_1*1000.0);
			Right_WS_Reduced.phase_b = (int)(Right_WS.Phase_2*1000.0);
			Right_WS_Reduced.motor_temp = (int)(Right_WS.MotorTemp*1000.0);
			Right_WS_Reduced.heatsink_temp = (int)(Right_WS.HeatsinkTemp*1000.0);
			Right_WS_Reduced.amp_hours = (int)(Right_WS.DCBusAHr*1000.0);	
			Right_WS_Reduced.odometer = (int)(Right_WS.Odometer*1000.0);	
		
			mlink_send_ws_reduced(&Left_WS_Reduced, MLINK_LHS_MOTOR_OFFSET);
			mlink_send_ws_reduced(&Right_WS_Reduced, MLINK_RHS_MOTOR_OFFSET);
			ws_sendout_timer = sc_get_timer();
		}
		
		//LEDs Flashing Loop
		if((sys_state != SYS_INIT) && (sc_get_timer() >= leds_flash_timer + LEDS_FLASH_MS)) {
			clockout_leds(1, &steveLEDs);
			leds_flash_timer = sc_get_timer();
		}
		
		if(sc_get_timer() >= state_confirm_timer + STATE_CONFIRM_MS) {
			horn_handler();
			brake_cutout_handler();
			state_confirm_timer = sc_get_timer();
		}
		
		//Wavesculptor - 10Hz Loop
		if(sc_get_timer() >= hundred_ms_timer + 100) {
			hundred_ms_timer = sc_get_timer();
			
			if(GPIO_GetValue(BRAKE_CUTOUT_PORT, BRAKE_CUTOUT_BIT)) {
				brake_cutout_state = BRAKE_CUTOUT_ON;
			} else {
				brake_cutout_state = BRAKE_CUTOUT_OFF;
			}
			
			data = sys_state;
			mlink_send(0x00, MLINK_SYS_STATE, (void*) &data);
			
			if(sys_state != SYS_INIT) {
				//Update current rpm - average rpm from both wavesculptors.
				if(USE_AVERAGE_VELOCITY) {
					current_rpm = (Left_WS.MotorVelocity + Right_WS.MotorVelocity)/2;
				} else {
					current_rpm = max(Left_WS.MotorVelocity, Right_WS.MotorVelocity);
				}
				
				current_velocity = mps2kph(rpm2mps(current_rpm, WHEEL_DIAMETER));
				uint32_t disp_current_velocity = (int)current_velocity;
				if (disp_current_velocity >= 100) {
					disp_current_velocity = disp_current_velocity - 100;
				}
				
				seg_dual_int(0, disp_current_velocity);
				
				data = (int)(current_velocity*1000.0);
				mlink_send(0x00, MLINK_CURRENT_VELOCITY, (void*) &data);
				
				data = (int)((mps2kph(rpm2mps(cruise_rpm, WHEEL_DIAMETER)))*1000.0);
				mlink_send(0x00, MLINK_CRUISE_VELOCITY, (void*) &data);

				data = (int)(cruise_proportional*1000.0);
				mlink_send(0x00, MLINK_CRUISE_PROPORTIONAL, (void*) &data);
				
				data = (int)(cruise_integral*1000.0);
				mlink_send(0x00, MLINK_CRUISE_INTEGRAL, (void*) &data);
				
			}
		
			//CRUISE MODE
			if(sys_state == SYS_CRUISE && precharge_state == PRECHARGED && brake_cutout_state == BRAKE_CUTOUT_OFF) {
				steveLEDs.led[0]->state = ON;
				steveLEDs.led[0]->flash = OFF;
				steveLEDs.led[1]->state = OFF;
				steveLEDs.led[1]->flash = OFF;
				steveLEDs.led[2]->state = OFF;
				steveLEDs.led[2]->flash = OFF;
				steveLEDs.led[3]->state = OFF;
				steveLEDs.led[3]->flash = OFF;
				steveLEDs.led[4]->state = OFF;
				steveLEDs.led[4]->flash = OFF;
				steveLEDs.led[5]->state = OFF;
				steveLEDs.led[5]->flash = OFF;
				steveLEDs.led[6]->state = OFF;
				steveLEDs.led[6]->flash = OFF;
				steveLEDs.led[7]->state = ON;
				steveLEDs.led[7]->flash = OFF;

				cruise_torque = calculateTorque();
				
                                left_ws_state = LEFT_WS_ENABLE;
                                right_ws_state = RIGHT_WS_ENABLE;
                                steveLEDs.K5V[2]->state = OFF;
                                steveLEDs.K5V[2]->flash = OFF;
                                steveLEDs.K5V[3]->state = OFF;
                                steveLEDs.K5V[3]->flash = OFF;

				torqueControl(TORQUE_MODE_OPEN, (int32_t)cruise_rpm, cruise_torque, &Left_WS, &Right_WS);
				UART_printf("Cruise rpm: %d, current rpm: %d, torque: %d, vel: %d, cvel: %d\n\r", 
								(int)cruise_rpm, (int)current_rpm, (int)(cruise_torque*1000), (int)(current_velocity*1000),
								(int)(cruise_velocity*1000));
				cruise_velocity = mps2kph(rpm2mps(cruise_rpm, WHEEL_DIAMETER));	
				uint32_t disp_cruise_velocity = (int)cruise_velocity;
				if(disp_cruise_velocity >= 100) {
					disp_cruise_velocity = disp_cruise_velocity - 100;
				}
				 seg_dual_int (2, disp_cruise_velocity);
				 
				 //Cutout Mode
				if(massage_paddle(LEFT_PADDLE) >= 0.05) {
					sys_state = SYS_MANUAL;
					steveLEDs.K5V[6]->state = OFF;
					steveLEDs.K5V[6]->flash = OFF;
					steveLEDs.K5V[11]->state = OFF;
					steveLEDs.K5V[11]->flash = OFF;
				}
			}
		
		
			//MANUAL MODE
			if(sys_state == SYS_MANUAL && precharge_state == PRECHARGED) {
				steveLEDs.led[0]->state = OFF;
				steveLEDs.led[0]->flash = OFF;
				steveLEDs.led[1]->state = OFF;
				steveLEDs.led[1]->flash = OFF;
				steveLEDs.led[2]->state = OFF;
				steveLEDs.led[2]->flash = OFF;
				steveLEDs.led[3]->flash = ON;
				steveLEDs.led[4]->flash = ON;
				steveLEDs.led[5]->state = OFF;
				steveLEDs.led[5]->flash = OFF;
				steveLEDs.led[6]->state = OFF;
				steveLEDs.led[6]->flash = OFF;
				steveLEDs.led[7]->state = OFF;
				steveLEDs.led[7]->flash = OFF;
				
				float accel_float;
				float regen_float;
				float torque;
				
				accel_float = massage_paddle(RIGHT_PADDLE);
				regen_float = -massage_paddle(LEFT_PADDLE);
				
				/*Reconciliate Regen & Accel */
				if (regen_float < -0.01) {
					torque = regen_float;
					regen_state = REGEN_ON;
				} else {
					if(brake_cutout_state == BRAKE_CUTOUT_OFF || DRIVE_CUTOUT_OVER_RIDE) {
						torque = accel_float;
						regen_state = REGEN_OFF;
					} else {
						accel_float = 0.0;
						torque = 0.0;
					}
				}
				
				//Adjust torque vectoring.
				UART_printf("AF: %d\%% RF: %d\%% TRQ: %d\%%\n\r", 
					(int)(accel_float*100), (int)(regen_float*100), (int)(torque*100));
				
				if(precharge_state == PRECHARGED) {
					if(reverse_state == REVERSE_ON) {
						torqueControl(TORQUE_MODE_OPEN, -150, torque, &Left_WS, &Right_WS);
					} else {
						torqueControl(TORQUE_MODE_OPEN, 150, torque, &Left_WS, &Right_WS);
					}
					seg_dual_float (2, accel_float);
				}
				
			}
			
		}
		
		
		/* Send a UART and CAN message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			one_sec_timer = sc_get_timer();
			
			but0 = read_all_buttons();
			
			if (tog_bit == 0) {
				tog_bit = 1;
			} else {
				tog_bit = 0;
			}
			
			/* Send the message */
			UART_printf("1 second timer %u: but: %d\n\r", (unsigned int)one_sec_timer, but0);

			/* Twiddle the LEDs */
			toggle_red_led();
			toggle_yellow_led();
			
		}

		/* Tickle the watchdog so we don't reset */
		//WDT_Feed();
	}
}

float massage_paddle(uint32_t paddle_channel) {

	uint32_t paddle;
	float paddle_float;
	
	paddle = get_ADC(paddle_channel);
	paddle_float = PADDLE_MAX-paddle;
	paddle_float /= PADDLE_MAX;
	
	paddle_float -= 0.05;
	
	if(paddle_float <= 0.0) {
		paddle_float = 0.0;
	}
	
	paddle_float /= 0.90;
	
	if(paddle_float >= 1.0) {
		paddle_float = 1.0;
	}
	
	return paddle_float;
}

/* Pass targetRPM and desiredTorque, regen if targetRPM and desiredTorque sign different */
int32_t torqueControl(uint8_t controlMode, int32_t targetRPM, float desiredTorque, Wavesculptor_Output_Struct *ws_l_struct, Wavesculptor_Output_Struct *ws_r_struct){
  #define WS20_SPEED_LIMIT  1000
  #define WS22_SPEED_LIMIT  20000
  float WS20_Speed;
  float W22_Speed;
  int32_t deviceType;
  float sign=1;
  float leftTorque = 0.0;
  float rightTorque = 0.0;
  float slip;
  
  /* Note: Slip ratio can be calculated by (LeftSpeed / (LeftSpeed + RightSpeed)) or vice versa 0:All right, 0.5:Balanced ,1:All Left */
  
  if ( (controlMode == TORQUE_MODE_REGEN) | (controlMode == TORQUE_MODE_OPEN) ){
  
    leftTorque = absoluteFloat(desiredTorque);
    rightTorque = absoluteFloat(desiredTorque);
    
    /* Sign = sign of targetRPM if Sign of targetRPM and desiredTorque are different, regen */
    if(targetRPM > 0){
      sign = 1;
      if (desiredTorque > 0){
        /* Positive RPM and Torque = forward acceleration */
      } else {
        /* Positive RPM and negative Torque = forward regen */
          controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }
    }else{
      sign = -1;
	  //I have a feeling it is the other way around.
      if (desiredTorque > 0){
        /* Negative RPM and positive Torque = reverse regen */
          //controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }else{
        /* Negative RPM and negative Torque = reverse acceleration */
		  controlMode = TORQUE_MODE_REGEN;
      }
    }
    
  } 
  
  /* Setting set speed limits to wavesculptor types */
  if ( controlMode == TORQUE_MODE_REGEN){
    WS20_Speed = 0;
    W22_Speed = 0;
  } else {
    WS20_Speed = WS20_SPEED_LIMIT;
    W22_Speed = WS22_SPEED_LIMIT;
  }
  
    
  /* Set motor speed limit depending on type of motor controller, WS22 is in RPM, WS20 is in m/s - send roughly RPM/4 */
  deviceType = check_device_type(ws_l_struct);
  
  if(deviceType == WS_22 && left_ws_state == LEFT_WS_ENABLE){
    send_ws_drive_commands(sign*W22_Speed, leftTorque, (float) 1.0, ws_l_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_Speed, leftTorque, (float) 1.0, ws_l_struct);
  }


  deviceType = check_device_type(ws_r_struct);

  if(deviceType == WS_22 && right_ws_state == RIGHT_WS_ENABLE){
    send_ws_drive_commands(sign*W22_Speed, rightTorque, (float) 1.0, ws_r_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_Speed, rightTorque, (float) 1.0, ws_r_struct);
  }

  /* Send command to each WS: send_ws_drive_commands(sign*MAX_SPEED, torque_command, (float)0.3, ws_1_struct); */
  return 0;
}

float absoluteFloat(float input){
  
  if(input > 0){
    return input;
  } else if (input < 0) {
    return (-1.0*input);
  }
  
  return 0.0;
}

float limit(float input, float limitAmount){
  if(input > limitAmount){
    input = limitAmount;
  }
  return input;
}

/*MAX6955 Helpers*/
void seg_dual_float (uint8_t init_digit, float data) {
	seg_dual_int(init_digit, (int)(data*99));
}

void seg_dual_int (uint8_t init_digit, uint8_t integer) {
	seg_write_char(init_digit, integer/10);
	seg_write_char(init_digit+1, integer%10);
}

void seg_setup(void) {
	I2CWrite_3(SEG_ADDR_WRITE, SEG_CONTROL_REGISTER, 0x0D);
	I2CWrite_3(SEG_ADDR_WRITE, SEG_GLOBAL_INTENSITY, 0x0F);
	I2CWrite_3(SEG_ADDR_WRITE, SEG_SCAN_LIMIT, 0x03);
	I2CWrite_3(SEG_ADDR_WRITE, SEG_DIGIT_TYPE, 0xFF);
}

void seg_write_char(uint8_t digit, uint8_t font) {
	if(digit >= 0 && digit <= 3){
		I2CWrite_3(SEG_ADDR_WRITE, SEG_DIGIT_BASE+digit, font);
	}
	else {
		UART_printf("SEG Digit Overlimit: %d\n\r",digit);
	}
}

void I2CWrite_3(uint8_t addr, uint8_t reg, uint8_t data) {

	I2CWriteLength = 3;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = addr;
	I2CMasterBuffer[1] = reg;
	I2CMasterBuffer[2] = data;
	I2CEngine();
}

/* Reading all buttons */
int read_all_buttons(void) {

	int ret_buttons = 0;
	ret_buttons |= GPIO_GetValue(B0_PORT, B0_BIT);
	ret_buttons |= GPIO_GetValue(B1_PORT, B1_BIT) << 1;
	ret_buttons |= GPIO_GetValue(B2_PORT, B2_BIT) << 2;
	ret_buttons |= GPIO_GetValue(B3_PORT, B3_BIT) << 3;
	ret_buttons |= GPIO_GetValue(B4_PORT, B4_BIT) << 4;
	ret_buttons |= GPIO_GetValue(B5_PORT, B5_BIT) << 5;
	ret_buttons |= GPIO_GetValue(B6_PORT, B6_BIT) << 6;
	ret_buttons |= GPIO_GetValue(B7_PORT, B7_BIT) << 7;
	ret_buttons |= GPIO_GetValue(B8_PORT, B8_BIT) << 8;
	ret_buttons |= GPIO_GetValue(B9_PORT, B9_BIT) << 9;
	ret_buttons |= GPIO_GetValue(B10_PORT, B10_BIT) << 10;
	ret_buttons |= GPIO_GetValue(B11_PORT, B11_BIT) << 11;

	return ret_buttons;
}



float calculateTorque(void) {
	float error;
	//float cruise_proportional;
	float torque;
	
	error = cruise_rpm - current_rpm;
	cruise_proportional = error * PROPORTIONAL_GAIN;
	cruise_integral = cruise_integral + error * INTEGRAL_GAIN;
	
	cruise_proportional = absLimit(cruise_proportional, 1.0);
	cruise_integral = absLimit(cruise_integral, 1.0);
	
        if (CRUISE_REGEN_DISABLE && cruise_integral <= 0){
                cruise_integral = 0;
        }

	torque = absLimit(cruise_proportional + cruise_integral, 1.0);
	
	if (CRUISE_REGEN_DISABLE && torque <= 0){
		torque = 0;
        }

	return torque;
	
	
}

float absLimit(float input, float limit){
	float returnVal = input;
	
	if(input > 0){
		if(input > limit){
			returnVal = limit;
		}
	} else {
		if(input < -limit){
			returnVal = -1.0*limit;
		}
	}
	
	return returnVal;
}

float max(float left, float right) {
        if (abs(left) > abs(right)) {
                return left;
        } else {
                return right;
        }
}

float abs(float input) {
        if (input < 0) {
                return -input;
        } else {
                return input;
        }
}
