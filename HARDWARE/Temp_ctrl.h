#ifndef _TEMP_CTRL_H
#define _TEMP_CTRL_H

#include "sys.h"

#define BYTE_ALIGN   __attribute__ ((packed))  //���ֽڶ���

//typedef struct PID
//{
//	volatile float p;
//	volatile float i;
//	volatile float d;
//	float err_curr;   // ��ǰ���
//	float err_last;  // �ϴ����
//	float err_sum;   // ����  ����
//	float err_last_last;  // ���ϴε����
//	float target;    // Ŀ��ֵ
//	float Out;  // ���� �����ֵ
//}PIDDataDef;

typedef struct{
	
	float ek0;        // 当前误差
	float ek1;        // 上一次误差
	float ek2;        // 上两次误差
	float kp;         // 比例系数
	float ki;         // 积分系数
	float kd;         // 微分系数
	float LocSum;     // 积分累加和
	float ts;         // 采样时间(ms)
	float last_output;// 上一次输出值

} PIDDataDef;

float PID_Incremental(PIDDataDef *pid,float target,float current);
void PIDStructInit(PIDDataDef *pid,float p,float i,float d);
u16 TempPID_LocCAL(PIDDataDef *pid,u16 SV_Temp,float PV_Temp);  //λ��ʽ //SV�趨ֵ PVʵ��ֵ
u16 TempPID_FreqCAL(PIDDataDef *pid,float set_temp,float current_temp,u16 min_freq,u16 max_freq,float deadband);
#endif
