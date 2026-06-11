#ifndef	__DOOR_CTRL_H_ 
#define __DOOR_CTRL_H_ 

//door lock state 0:lock, 1:unlock
#define DOOR_LOCK_STATE_GPIO  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
//case state 0:close 1:open
#define	DOOR_CASE_STATE_GPIO	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3);

#define DOOR_FRESH_TICK_1MS 10

#define CLOSE_DELAY 500	//500ms

#define OPEN_DELAY_TOTAL 2200	//2.4s unlock run time + wait time
#define OPEN_DELAY_WAIT 1500	//1.5s


#define DOOR_DELAY_TIME(x) (DOOR_FRESH_TICK_1MS*x)  

#define UNLOCKING 	1
#define LOCKING 	2
typedef struct
{
	unsigned char lock_action;
	unsigned char unlock_action;
	unsigned char cw_spd_action;
	unsigned char ccw_spd_action;
	unsigned char lock_state;
	unsigned char case_state;
	unsigned int timer_ms;
    unsigned int rtu_data_case;
    unsigned int rtu_data_lock;
	unsigned int rtu_data_action;
}DoorDataDef;


void Door_Fresh_Timer(DoorDataDef  *door);
void DC_Motor_Init(void);
void Door_Dispose(DoorDataDef  *door);



#endif
