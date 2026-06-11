#ifndef _TEMP_CTRL_H
#define _TEMP_CTRL_H

#include "sys.h"

#define BYTE_ALIGN   __attribute__ ((packed))  //单字节对齐

//typedef struct PID
//{
//	volatile float p;
//	volatile float i;
//	volatile float d;
//	float err_curr;   // 当前误差
//	float err_last;  // 上次误差
//	float err_sum;   // 误差和  积分
//	float err_last_last;  // 上上次的误差
//	float target;    // 目标值
//	float Out;  // 增量 输出的值
//}PIDDataDef;

typedef struct{
	
	float ek0;
	float ek1;
	float ek2;
	float kp;
	float ki;
	float kd;
	float LocSum;

} PIDDataDef;

float PID_Incremental(PIDDataDef *pid,float target,float current);
void PIDStructInit(PIDDataDef *pid,float p,float i,float d);
u16 TempPID_LocCAL(PIDDataDef *pid,u16 SV_Temp,float PV_Temp);  //位置式 //SV设定值 PV实测值
#endif
