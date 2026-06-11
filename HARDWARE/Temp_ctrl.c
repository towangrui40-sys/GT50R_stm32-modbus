#include "Temp_ctrl.h"
#include "includes.h"



// pid结构体的初始化
void PIDStructInit(PIDDataDef *pid,float p,float i,float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;

	pid->ek0 = 0;
	pid->ek1 = 0;
	pid->ek2 = 0;
	pid->LocSum = 0;
}




u16 TempPID_LocCAL(PIDDataDef *pid,u16 SV_Temp,float PV_Temp)  //位置式 //SV设定值 PV实测值
{
	float Proportional,Integral,Differential;
	s16 PIDLoc;
	
	pid->ek0 = SV_Temp - PV_Temp;
	if(pid->ek0 < 0) pid->ek0 = 0 - pid->ek0;
	
	pid->LocSum += pid->ek0;
	Proportional =  pid->kp*pid->ek0;
	Integral = pid->ki*pid->LocSum;
	Differential = pid->kd*(pid->ek1 - pid->ek0);
	pid->ek1 = pid->ek0;
	
	if(Proportional<=0)
	{
		Proportional = 0;
	}
	if(Integral > 100)
	{
		Integral  = 100;
	}
	if(Integral < -100)
	{
		Integral  = -100;
	}
	PIDLoc =Proportional+Integral+Differential;
	
	if(PIDLoc>200)
	{
		PIDLoc = 200;
	}

	if(PV_Temp < SV_Temp)
	{
		PIDLoc = 0;
		pid->ek0 = 0;
		pid->ek1 = 0;
		pid->ek2 = 0;
		pid->LocSum = 0;
	}
	
	return PIDLoc;
}
