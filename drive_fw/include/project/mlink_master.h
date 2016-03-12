#define MLINK_CLK_DIV			8
#define MLINK_START_SYNC_PORT	3
#define MLINK_START_SYNC_BIT	2
#define MLINK_ACK_PORT			3
#define MLINK_ACK_BIT			3
#define MLINK_PORT				1

//MLINK Registers
#define MLINK_PRECHARGE_ADDR			0x01
#define MLINK_PRECHARGE_CHARGE			0x02
#define	MLINK_PRECHARGE_DISCHARGE		0x01

#define MLINK_LEFT_IND_ADDR				0x02
#define MLINK_LEFT_IND_OFF				0x00
#define MLINK_LEFT_IND_ON				0x01

#define MLINK_RIGHT_IND_ADDR			0x03
#define MLINK_RIGHT_IND_OFF				0x00
#define MLINK_RIGHT_IND_ON				0x01

#define MLINK_HAZARDS_ADDR				0x04
#define MLINK_HAZARDS_OFF				0x00
#define MLINK_HAZARDS_ON				0x01

#define MLINK_HORN_ADDR					0x05
#define MLINK_HORN_OFF					0x00
#define MLINK_HORN_ON					0x01

#define MLINK_BRAKE_LIGHT_ADDR			0x06
#define MLINK_BRAKE_LIGHT_OFF			0x00
#define MLINK_BRAKE_LIGHT_ON			0x01

#define MLINK_CURRENT_VELOCITY			0x07
#define MLINK_SYS_STATE					0x08
#define MLINK_CRUISE_VELOCITY			0x09
#define MLINK_CRUISE_PROPORTIONAL		0x10
#define MLINK_CRUISE_INTEGRAL			0x11

//Hack WS Heatsink TEMP
//#define MLINK_WS_L_HEATSINK_TEMP		0x20
//#define MLINK_WS_R_HEATSINK_TEMP		0x21

//Left Hand Side Motor
#define MLINK_LHS_MOTOR_OFFSET			0x20

//Right Hand Side Motor
#define MLINK_RHS_MOTOR_OFFSET			0x40

//Wavesculptor Reduced
#define WS_TYPE							0
#define WS_ACTIVE_MOTOR					1
#define WS_ERRORS						2
#define WS_LIMIT						3
#define WS_BUS_CURRENT					4
#define WS_BUS_VOLT						5
#define WS_MOTOR_VEL					6
#define WS_PHASE_A						7
#define WS_PHASE_B						8
#define WS_MOTOR_TEMP					9
#define WS_HEATSINK_TEMP				10
#define WS_AMP_HOURS					11
#define WS_ODOMETER						12

void mlink_setup(void);
void mlink_send(uint8_t control, uint8_t address, uint8_t data[4]);
void mlink_send_ws_reduced(ws_reduced_t *WS_Reduced, uint32_t address);