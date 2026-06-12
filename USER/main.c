#include "includes.h"
#include "mb.h"
#include "mb_m.h"
#include "user_mb_app.h"
#include "Temp_ctrl.h"



//RPM 1300-1750   26HZ~70HZ
//℃  -50~25

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
#define FREQ_MIN            35      // nominal: 35Hz
#define FREQ_MAX            160     // nominal: 160Hz
#define FREQ_TOLERANCE      2       // table tolerance: +/-2Hz
#define FREQ_ACCEPT_MIN     (FREQ_MIN - FREQ_TOLERANCE)
#define FREQ_ACCEPT_MAX     (FREQ_MAX + FREQ_TOLERANCE)
#define COMPRESSOR_RPM_MIN  1400
#define COMPRESSOR_RPM_MAX  4500
#define TEMP_PID_KP         8.0f
#define TEMP_PID_KI         0.08f
#define TEMP_PID_KD         2.0f
#define TEMP_PID_DEADBAND   0.5f
#define SAMPLE_TIME  0.1f

SysDataDef sys;
volatile uint16_t g_compressor_target_rpm = 0; //PID计算得到的目标转速，未经过转速容错和范围限制处理
volatile uint16_t g_compressor_pid_freq = 0;  //PID计算得到的频率，未经过频率容错和范围限制处理
extern int modbus_flag;
//====================  ====================
void Init_Rtu_RegHoldingBuf(SysDataDef* dat)
{
	memset(&sys,0,sizeof(sys));
	usRegHoldingBuf[ADDR_MOTOR_GET_FWD].data = (u16 *)&dat->fwd;
	usRegHoldingBuf[ADDR_MOTOR_SET_VOL].data = (u16 *)&dat->set_vol;
	usRegHoldingBuf[ADDR_SET_TEMP_CMD].data = (u16 *)&dat->set_temp_cmd;
	usRegHoldingBuf[ADDR_SET_TEMP].data = (u16 *)&dat->set_temp;   
	usRegHoldingBuf[ADDR_NTC_TEMP].data = (u16 *)&dat->ntc_temp;
	usRegHoldingBuf[ADDR_DISP_FREQ].data = (u16 *)&dat->disp_freq;
  	usRegHoldingBuf[ADDR_DOOR].data = (u16 *)&dat->door;
	usRegHoldingBuf[ADDR_LOCK].data = (u16 *)&dat->lock;

	usRegHoldingBuf[ADDR_SET_LOCK].data = (u16 *)&dat->set_lock;  //门锁控制
	usRegHoldingBuf[ADDR_SET_DC2].data = (u16 *)&dat->set_dc2;
	usRegHoldingBuf[ADDR_SET_DC3].data = (u16 *)&dat->set_dc3;

	usRegHoldingBuf[ADDR_SET_AC1].data = (u16 *)&dat->set_ac1;   //压缩机风扇控制
	usRegHoldingBuf[ADDR_SET_AC2].data = (u16 *)&dat->set_ac2;
	usRegHoldingBuf[ADDR_SET_AC3].data = (u16 *)&dat->set_ac3;
	usRegHoldingBuf[ADDR_SET_FEQ].data = (u16 *)&dat->set_feq;
}

#define START_TASK_PRIO      			10
#define LED_TASK_PRIO       			9	 
#define MONITOR_TASK_PRIO				8
#define CONTROL_TASK_PRIO				7
#define HMI_INTERFACE_TASK_PRIO	    	5
#define COM_INTERFACE_TASK_PRIO		    3

#define START_STK_SIZE  			64
#define COM_INTERFACE_STK_SIZE		256
#define HMI_INTERFACE_STK_SIZE		256
#define MONITOR_STK_SIZE			256
#define CONTROL_STK_SIZE			128
#define LED_STK_SIZE  		    	128

OS_STK START_TASK_STK[START_STK_SIZE];

OS_STK COM_INTERFACE_TASK_STK[COM_INTERFACE_STK_SIZE];
OS_STK HMI_INTERFACE_TASK_STK[HMI_INTERFACE_STK_SIZE];
OS_STK MONITOR_TASK_STK[MONITOR_STK_SIZE];
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
OS_STK LED_TASK_STK[LED_STK_SIZE];
//OS_TMR   * tmr_adc;
//OS_TMR   * tmr_upload;
//OS_TMR	 * tmr_ctrl;
//OS_TMR   * tmr_spk;

void start_task(void *pdata);
void com_interface_task(void *pdata);
void hmi_interface_task(void *pdata);
void monitor_task(void *pdata);
void control_task(void *pdata);
void led_task(void *pdata);	
void TIM3_Configuration(uint32_t freq);
uint8_t Set_PWM_Frequency(uint32_t freq);
void Stop_PWM_Output(void);
uint32_t Normalize_Compressor_Freq(int16_t cmd_freq);
uint16_t Compressor_Rpm_By_Freq(uint32_t freq);

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
	NVIC_Configuration();
	Init_Rtu_RegHoldingBuf(&sys);
	PVD_Config();
	GPIO_Config();
	ADC1_Init();
	DAC1_Init();
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );
	OSStart();
 }

void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
	delay_init();
	
	OS_ENTER_CRITICAL();
	
 	OSTaskCreate(hmi_interface_task,(void *)0,(OS_STK*)&HMI_INTERFACE_TASK_STK[HMI_INTERFACE_STK_SIZE-1],HMI_INTERFACE_TASK_PRIO);
 	OSTaskCreate(com_interface_task,(void *)0,(OS_STK*)&COM_INTERFACE_TASK_STK[COM_INTERFACE_STK_SIZE-1],COM_INTERFACE_TASK_PRIO);
	OSTaskCreate(monitor_task,(void *)0,(OS_STK*)&MONITOR_TASK_STK[MONITOR_STK_SIZE-1],MONITOR_TASK_PRIO);
	OSTaskCreate(control_task,(void *)0,(OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],CONTROL_TASK_PRIO);
	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);
	OSTaskSuspend(START_TASK_PRIO);
	OS_EXIT_CRITICAL();
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
		//sys.ntc_temp= Get_Temp_10NTC(temp);

		if(DOOR == 0)sys.door = 1;
			else sys.door = 0;
		if(LOCK == 0)sys.lock = 1;
			else sys.lock = 0;
		
		//dc out
		if(sys.set_lock == 1) DC_CTRL_EX(ON);
		else DC_CTRL_EX(OFF);	
		if(sys.set_dc2 == 1) DC_CTRL2(ON);
		else DC_CTRL2(OFF);
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
		if(sys.set_vol != vol_temp)
		{
			//dac out
			temp = sys.set_vol;
			temp = temp/1.839f;
			DAC1_Set_Vol((u16)temp);
			vol_temp = sys.set_vol;		
		}
		// motor ctrl logit
		MC_FWD(sys.fwd);
		delay_ms(100);
	}
}
/******************************************
 * control_task
 * Description: control compressor frequency by PID
******************************************/
void control_task(void *pdata)
{
	PIDDataDef temp_pid;
	uint32_t target_freq;
	uint32_t last_freq = 0xFFFFFFFF;
	PIDStructInit(&temp_pid, TEMP_PID_KP, TEMP_PID_KI, TEMP_PID_KD);
	TIM3_Configuration(FREQ_MIN);
	delay_ms(1000);
	while(1)
	{
        if(sys.set_temp_cmd == 1) //temp_cmd为1开启控温 temp_cmd为0则关闭控温，直接使用手动设定的频率
        {
			g_compressor_pid_freq = TempPID_FreqCAL(&temp_pid,(float)sys.set_temp,(float)sys.ntc_temp,FREQ_MIN,FREQ_MAX,TEMP_PID_DEADBAND);
            // PID自动控温模式
            target_freq = Normalize_Compressor_Freq((int16_t)g_compressor_pid_freq);
        }
        else
        {
            // 手动频率模式
            target_freq = Normalize_Compressor_Freq(sys.set_feq); // 直接关闭压缩机输出，或者也可以改为使用sys.set_freq进行手动频率控制，视需求而定
        }
        
        sys.disp_freq = target_freq; // 将频率转换为转速进行显示，未经过转速容错和范围限制处理

		if(target_freq != last_freq)
		{
			if(target_freq == 0)
			{
				Stop_PWM_Output();
			}
			else
			{
				Set_PWM_Frequency(target_freq);
			}
			last_freq = target_freq;
		}
		delay_ms(500);
	}
}



/******************************************
 * led_task
 * Description: flash led to indicate system is running
******************************************/
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
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    period_value = (10000 / freq) - 1;
    
    if(period_value > 65535) period_value = 65535;
    if(period_value < 1) period_value = 1;
    
    TIM_TimeBaseStructure.TIM_Period = period_value;      // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;       // Ԥ��Ƶֵ���õ�10kHz����Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    TIM_OCInitStructure.TIM_Pulse = (period_value + 1) / 2;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    
    TIM_Cmd(TIM3, ENABLE);
}
/**
 * Set_PWM_Frequency
 * Description: 设置PWM频率，频率范围受限于预设的最小值和最大值，超出范围将返回错误
 * Parameters:
 *   - freq: 期望设置的频率值，单位为Hz
 * Returns:
 *   - 0: 成功设置频率
 *   - 1: 频率值超出可接受范围，未设置频率
*/
uint8_t Set_PWM_Frequency(uint32_t freq)
{
    uint32_t period_value;
	// nominal range: 35Hz ~ 160Hz
    if(freq < FREQ_MIN || freq > FREQ_MAX)
		return 1;
    period_value = (10000 / freq) - 1;
    TIM3->ARR = period_value;
    TIM3->CCR1 = (period_value + 1) / 2;
    return 0;
}

void Stop_PWM_Output(void)
{
	TIM3->CCR1 = 0;
}
/*
 * Normalize_Compressor_Freq
 * Description: 将PID计算得到的频率进行容错和范围限制处理，确保输出给压缩机的频率在可接受范围内
 * Parameters:
 *   - cmd_freq: PID计算得到的原始频率值，单位为Hz，可能超出可接受范围
 * Returns:
 *   - 处理后的频率值，单位为Hz，如果输入频率超出可接受范围则返回0表示关闭压缩机输出，否则返回限制在FREQ_MIN和FREQ_MAX之间的频率值
*/
uint32_t Normalize_Compressor_Freq(int16_t cmd_freq)  // 将PID计算得到的频率进行容错和范围限制处理，确保输出给压缩机的频率在可接受范围内
{
	if(cmd_freq < FREQ_ACCEPT_MIN || cmd_freq > FREQ_ACCEPT_MAX)
	{
		return 0;
	}
	if(cmd_freq < FREQ_MIN)
	{
		return FREQ_MIN;
	}
	if(cmd_freq > FREQ_MAX)
	{
		return FREQ_MAX;
	}
	return (uint32_t)cmd_freq;
}

/*
 * Compressor_Rpm_By_Freq
 * Description: 将PID计算得到的频率转换为压缩机转速，未经过转速容错和范围限制处理
 * Parameters:
 *   - freq: PID计算得到的频率值，单位为Hz
 * Returns:
 *   - 转速值，单位为RPM，如果输入频率为0则返回0
 */
uint16_t Compressor_Rpm_By_Freq(uint32_t freq)  // 将PID计算得到的频率转换为压缩机转速，未经过转速容错和范围限制处理
{
	if(freq == 0)
	{
		return 0;
	}
	if(freq <= 47)
	{
		return COMPRESSOR_RPM_MIN;
	}
	if(freq >= 150)
	{
		return COMPRESSOR_RPM_MAX;
	}
	return (uint16_t)(freq * 30U);
}

