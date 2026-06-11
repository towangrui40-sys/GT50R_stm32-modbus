	
#include "includes.h"
#include "mb.h"
#include "mb_m.h"
#include "user_mb_app.h"

 /***********************Version record********************************
 
 Version:V1.0
 date:2022/06/20
 author:Jason
 Note:
 1.Initial creation 
 2.Add rtos:ucosii
 3.Add com protocal:freemodbus-RTU
 4.Add control logic
 5.Add monitor task
 6.Add hmi servo processing task
 ------------------------------------------ 
 
 
 *************************************************************/
 
#define FREQ_MIN    35      // 最小频率35Hz
#define FREQ_MAX    158     // 最大频率158Hz
#define FREQ_STOP   160     // 最大频率158Hz
#define SAMPLE_TIME  0.1f    // 采样时间0.1秒(100ms)
// 全局变量
SysDataDef sys;
extern int modbus_flag;
// ==================== PID控制器结构 ====================
void Init_Rtu_RegHoldingBuf(SysDataDef* dat)
{
	memset(&sys,0,sizeof(sys));
	usRegHoldingBuf[ADDR_MOTOR_GET_FWD].data = (u16 *)&dat->mc_para.fwd;
	usRegHoldingBuf[ADDR_MOTOR_SET_RUN].data = (u16 *)&dat->mc_para.set_run;
	usRegHoldingBuf[ADDR_MOTOR_SET_VOL].data = (u16 *)&dat->mc_para.set_vol;

	usRegHoldingBuf[ADDR_SET_TEMP].data = (u16 *)&dat->set_temp;   
	usRegHoldingBuf[ADDR_NTC_TEMP].data = (u16 *)&dat->ntc_temp;
	
  usRegHoldingBuf[ADDR_DOOR].data = (u16 *)&dat->door;
	usRegHoldingBuf[ADDR_LOCK].data = (u16 *)&dat->lock;
	
	usRegHoldingBuf[ADDR_SET_LOCK].data = (u16 *)&dat->set_lock;
	usRegHoldingBuf[ADDR_SET_DC2].data = (u16 *)&dat->set_dc2;
	usRegHoldingBuf[ADDR_SET_DC3].data = (u16 *)&dat->set_dc3;
	
	usRegHoldingBuf[ADDR_SET_AC1].data = (u16 *)&dat->set_ac1;
	usRegHoldingBuf[ADDR_SET_AC2].data = (u16 *)&dat->set_ac2;
	usRegHoldingBuf[ADDR_SET_AC3].data = (u16 *)&dat->set_ac3;
	usRegHoldingBuf[ADDR_SET_FEQ].data = (u16 *)&dat->set_feq;
}
/////////////////////////UCOSII任务堆栈设置///////////////////////////////////
/////////////////////////任务设置///////////////////////////////////			   
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
#define LED_TASK_PRIO       			9	 
#define MONITOR_TASK_PRIO					8
#define CONTROL_TASK_PRIO					7
#define HMI_INTERFACE_TASK_PRIO		5
#define COM_INTERFACE_TASK_PRIO		3

//设置任务堆栈大小
#define START_STK_SIZE  					64
#define COM_INTERFACE_STK_SIZE		256
#define HMI_INTERFACE_STK_SIZE		256
#define MONITOR_STK_SIZE					256
#define CONTROL_STK_SIZE					128
#define LED_STK_SIZE  		    		128
//任务堆栈
OS_STK START_TASK_STK[START_STK_SIZE];

OS_STK COM_INTERFACE_TASK_STK[COM_INTERFACE_STK_SIZE];
OS_STK HMI_INTERFACE_TASK_STK[HMI_INTERFACE_STK_SIZE];
OS_STK MONITOR_TASK_STK[MONITOR_STK_SIZE];
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
OS_STK LED_TASK_STK[LED_STK_SIZE];
//OS_TMR   * tmr_adc;			//软件定时器1
//OS_TMR   * tmr_upload;		//软件定时器2
//OS_TMR	 * tmr_ctrl; 
//OS_TMR   * tmr_spk;

//任务函数
void start_task(void *pdata);
void com_interface_task(void *pdata);
void hmi_interface_task(void *pdata);
void monitor_task(void *pdata);
void control_task(void *pdata);
void led_task(void *pdata);	
void TIM3_Configuration(uint32_t freq);
uint8_t Set_PWM_Frequency(uint32_t freq);
const float Rp_10K = 10000.0; //10K
const float Bx_10K = 3950.0;//B
const float T2_10K = (273.15+25);//T2
const float Ka_10K = 273.15;
float Get_Temp_10NTC(float res)
{
	float Rt;
	float temp;
	Rt = res;
	//like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)),  
	temp = Rt/Rp_10K;
	temp = (float)log(temp);//ln(Rt/Rp)
	temp/=Bx_10K;//ln(Rt/Rp)/B
	temp+=(1/T2_10K);
	temp = 1/(temp);
	temp-=Ka_10K;
	return temp;
}


 int main(void)
 {
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	Init_Rtu_RegHoldingBuf(&sys);
	PVD_Config();
	GPIO_Config();		  	 //初始化与LED连接的硬件接口
	ADC1_Init();
	DAC1_Init();
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();
 }

 //开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
	delay_init();	     //延时初始化	 
	
	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)   
	
 	OSTaskCreate(hmi_interface_task,(void *)0,(OS_STK*)&HMI_INTERFACE_TASK_STK[HMI_INTERFACE_STK_SIZE-1],HMI_INTERFACE_TASK_PRIO);
 	OSTaskCreate(com_interface_task,(void *)0,(OS_STK*)&COM_INTERFACE_TASK_STK[COM_INTERFACE_STK_SIZE-1],COM_INTERFACE_TASK_PRIO);
	OSTaskCreate(monitor_task,(void *)0,(OS_STK*)&MONITOR_TASK_STK[MONITOR_STK_SIZE-1],MONITOR_TASK_PRIO);
	OSTaskCreate(control_task,(void *)0,(OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],CONTROL_TASK_PRIO);
	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}	  

void hmi_interface_task(void *pdata)
{
#if ( (MB_RTU_ENABLED | MB_ASCII_ENABLED) > 0)	
	eMBInit(MB_RTU, 0x03, 1, 115200, MB_PAR_NONE);
	eMBEnable();	
#endif	
	while(1)
	{	
		eMBPoll();
		delay_ms(10);
	}
}

void com_interface_task(void *pdata)
{
	delay_ms(100);
//	uart2_init(115200);
	delay_ms(100);
	while(1)
	{
		delay_ms(1000);
	}
}

void monitor_task(void *pdata)
{
	static float adc_ch1=0;
	static short vol_temp=0;
	float temp=0;
	DAC1_Set_Vol((u16)temp);
	delay_ms(1000);

	while(1)
	{
			//get temprature
		adc_ch1	= Get_Adc(0,adc_ch1,0.5f); 
		temp = (float)adc_ch1/4096.0f;
		temp = 1/temp-1;
		temp = Rp_10K/temp;
		sys.ntc_temp= Get_Temp_10NTC(temp);

		if(DOOR == 0)sys.door = 1;
			else sys.door = 0;
		if(LOCK == 0)sys.lock = 1;
			else sys.lock = 0;
		
		//dc out
		if(sys.set_lock == 1) DC_CTRL_EX(ON);
		else DC_CTRL_EX(OFF);	
//		if(sys.set_dc2 == 1) DC_CTRL2(ON);
//		else DC_CTRL2(OFF);
		if(sys.set_dc3 == 1) DC_CTRL3(ON);
		else DC_CTRL3(OFF);
		//ac out
		if(sys.set_ac1 == 1) AC_CTRL1(ON);
		else AC_CTRL1(OFF);
		if(sys.set_ac2 == 1) AC_CTRL2(ON);
		else AC_CTRL2(OFF);
		if(sys.set_ac3 == 1) AC_CTRL3(ON);
		else AC_CTRL3(OFF);		
		//dac out
		if(sys.mc_para.set_vol != vol_temp)
		{
			//dac out
			temp = sys.mc_para.set_vol;
			temp = temp/1.839f;
			DAC1_Set_Vol((u16)temp);
			vol_temp = sys.mc_para.set_vol;		
		}
		// motor ctrl logit
		switch(sys.mc_para.set_run)
		{
			//fwd
			case 0x01:
					sys.mc_para.fwd = 1;
				break;
			case 0x04:
					sys.mc_para.fwd = 0;
				break;

			default:
				break;
		}

		MC_FWD(sys.mc_para.fwd);
		delay_ms(100);
	}
}
/******************************************
CW为顺时针
CCW为逆时针

电路板
fwd为正转 电机顺时针
rev为反转 电机逆时针
******************************************/
void control_task(void *pdata)
{;
	TIM3_Configuration(160);
	delay_ms(1000);
	
	while(1)
	{
		sys.set_ac1 = 1;
		Set_PWM_Frequency(FREQ_MAX);
		delay_ms(500);
	}
}



//LED任务
void led_task(void *pdata)
{
	static short delay=500;
	while(1)
	{	
		LED_RUN(ON);
		delay_ms(delay);
		LED_RUN(OFF);
		delay_ms(delay);
	}
}


void TIM3_Configuration(uint32_t freq)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint32_t period_value;
    
    // 使能TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 计算ARR值
    // 系统时钟72MHz，预分频7200-1 => TIM3时钟=10kHz
    // 频率 = 10kHz / (ARR + 1)
    // ARR = (10kHz / 频率) - 1
    period_value = (10000 / freq) - 1;
    
    // 限制ARR值范围（确保在16位范围内）
    if(period_value > 65535) period_value = 65535;
    if(period_value < 1) period_value = 1;
    
    // 配置TIM3时基
    TIM_TimeBaseStructure.TIM_Period = period_value;      // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;       // 预分频值，得到10kHz计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 配置TIM3输出比较通道1（PWM模式）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    // 设置50%占空比
    TIM_OCInitStructure.TIM_Pulse = (period_value + 1) / 2;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // 清除更新标志
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    
    // 启动TIM3
    TIM_Cmd(TIM3, ENABLE);
}

uint8_t Set_PWM_Frequency(uint32_t freq)
{
    uint32_t period_value;
    
    // 检查频率范围
    if(freq < FREQ_MIN || freq > FREQ_MAX)
        return 1;  // 频率超出范围
    
    // 计算ARR值
    // TIM3时钟 = 72MHz / 7200 = 10kHz
    period_value = (10000 / freq) - 1;
    
    // 更新ARR寄存器
    TIM3->ARR = period_value;
    
    // 更新CCR1值以保持50%占空比
    TIM3->CCR1 = (period_value + 1) / 2;
    
    
    return 0;  // 设置成功
}
