#ifndef __GPIO_H
#define __GPIO_H	 

#include "stm32f10x.h"

#define ON	1
#define OFF	0

	//================电机控制端口============
	
//MC_FWD
#define MC_FWD(a)	if (a)	\
					GPIO_ResetBits(GPIOC,GPIO_Pin_0);\
					else		\
					GPIO_SetBits(GPIOC,GPIO_Pin_0)	
//MC_REV
#define MC_REV(a)	if (a)	\
					GPIO_ResetBits(GPIOC,GPIO_Pin_13);\
					else		\
					GPIO_SetBits(GPIOC,GPIO_Pin_13)	
//MC_RESET
#define MC_RESET(a)	if (a)	\
					GPIO_ResetBits(GPIOB,GPIO_Pin_12);\
					else		\
					GPIO_SetBits(GPIOB,GPIO_Pin_12)		
//MC_ALARM
#define MC_ALARM	PAin(15)						

#define DOOR		PAin(0)

#define LOCK		PAin(1)			
	//================直流控制端口============
	
//DC_CTRL1
#define DC_CTRL1(a)	if (a)	\
					GPIO_ResetBits(GPIOA,GPIO_Pin_6);\
					else		\
					GPIO_SetBits(GPIOA,GPIO_Pin_6)
//DC_CTRL2
#define DC_CTRL2(a)	if (a)	\
					GPIO_ResetBits(GPIOA,GPIO_Pin_7);\
					else		\
					GPIO_SetBits(GPIOA,GPIO_Pin_7)
//DC_CTRL3
#define DC_CTRL3(a)	if (a)	\
					GPIO_ResetBits(GPIOA,GPIO_Pin_8);\
					else		\
					GPIO_SetBits(GPIOA,GPIO_Pin_8)
//DC_CTRL_EX  EX_GPIO1
#define DC_CTRL_EX(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_5)	

//LEDRUN
#define LED_RUN(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_3)	
//BEEP_OUT
#define BEEP_OUT(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_2)
					
//================AC控制端口============	
	
//AC_CTRL1
#define AC_CTRL1(a)	if (a)	\
					GPIO_ResetBits(GPIOC,GPIO_Pin_3);\
					else		\
					GPIO_SetBits(GPIOC,GPIO_Pin_3)
//AC_CTRL2
#define AC_CTRL2(a)	if (a)	\
					GPIO_ResetBits(GPIOC,GPIO_Pin_2);\
					else		\
					GPIO_SetBits(GPIOC,GPIO_Pin_2)
//AC_CTRL3
#define AC_CTRL3(a)	if (a)	\
					GPIO_ResetBits(GPIOC,GPIO_Pin_1);\
					else		\
					GPIO_SetBits(GPIOC,GPIO_Pin_1)
//AC_CTRL_EX  DAC2
#define AC_CTRL_EX(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_6);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_6)


void PVD_Config(void);
void GPIO_Config(void);//初始化
void Init_Motor_Gpio(void);					
void Init_Roller_Gpio(void);
		 				    
#endif
