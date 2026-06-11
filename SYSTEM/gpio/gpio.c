#include "gpio.h"


void PVD_Config(void)
{
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	PWR_PVDLevelConfig(PWR_PVDLevel_2V9);
	PWR_PVDCmd(ENABLE);
	
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //两级占先 八级副优先
	//中断分组设置
	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line16; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //使用中断模式?
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//电压低于阀值时产生中断?
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; // 使能中断线?
	EXTI_Init(&EXTI_InitStructure); // 初始
}


void GPIO_Config(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	 //使能PB,PE端口时钟
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	
	

/*-----------------------GPIO OUT CFG-------------------*/	
//PA6--DC_CTRL1 PA7--DC_CTRL2 PA8--DC_CTRL3 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz 	
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8;	
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz 	
 GPIO_Init(GPIOA, &GPIO_InitStructure);		
//PB3--LEDRUN  PB12--RESET
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_12;	
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化
	
//PC0--FWD  PC1--AC_CTRL3 PC2--AC_CTRL2 PC3--AC_CTRL1 PC13--REV
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_13;	
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化
 
 //PD2--BEEP
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
 GPIO_Init(GPIOD, &GPIO_InitStructure);					 //根据设定参数初始化

  
/*-----------------------GPIO IN CFG-------------------*/		
//PA0--DOOR PA1--LOCK PA15--ALARM
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_15;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO	
  
 //PC12 VBUS
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO	
/////////////////////////////////////////////////////////////////////
	MC_FWD(OFF); 
	MC_REV(OFF);
	MC_RESET(OFF);
	
	DC_CTRL1(OFF);
	DC_CTRL2(ON);
	DC_CTRL3(OFF);
	DC_CTRL_EX(OFF);
	
	AC_CTRL1(OFF);
	AC_CTRL2(OFF);
	AC_CTRL3(OFF);//AC FAN
	AC_CTRL_EX(OFF);
	
	LED_RUN(OFF);
	BEEP_OUT(OFF);
}

void PVD_IRQHandler(void)
{
	DC_CTRL1(OFF);
	DC_CTRL2(OFF);
	DC_CTRL3(OFF);
	EXTI_ClearITPendingBit(EXTI_Line16);
}


