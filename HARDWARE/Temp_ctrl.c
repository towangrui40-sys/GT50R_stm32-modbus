#include "Temp_ctrl.h"
#include "includes.h"



// pid�ṹ��ĳ�ʼ��
void PIDStructInit(PIDDataDef *pid,float p,float i,float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;

	pid->ek0 = 0;
	pid->ek1 = 0;
	pid->ek2 = 0;
	pid->LocSum = 0;
	pid->ts = 500.0f;      // 默认采样时间500ms
	pid->last_output = 0;
}





u16 TempPID_FreqCAL(PIDDataDef *pid,float set_temp,float current_temp,u16 min_freq,u16 max_freq,float deadband)
{
	float error;
	float proportional;
	float integral;
	float differential;
	float output;
	float max_output;
	float ts_normalized;  // 归一化采样时间
	float max_step;
	float delta;

	if(max_freq <= min_freq)
	{
		return min_freq;
	}

	// 制冷方向误差：当前温度高于设定温度时，才需要提升压缩机频率
	error = current_temp - set_temp;

	// 温度低于或等于设定值时，压缩机回到最低频率
	if(error <= 0.0f)
	{
		if(error <= -deadband)
		{
			pid->ek0 = 0;
			pid->ek1 = 0;
			pid->LocSum = 0;
		}
		pid->last_output = 0;
		return min_freq;
	}

	// 温度超出设定值但仍在死区内，维持最低频率，避免小幅振荡
	if(error < deadband)
	{
		pid->ek0 = 0;
		pid->ek1 = 0;
		pid->last_output = 0;
		return min_freq;
	}

	// 改进3：采样时间补偿 (ms -> s)
	ts_normalized = pid->ts / 1000.0f;

	pid->ek0 = error;
	pid->LocSum += pid->ek0 * ts_normalized;

	// 改进4：根据控制量范围合理设定积分限幅
	max_output = (float)(max_freq - min_freq);
	if(pid->LocSum > max_output)
	{
		pid->LocSum = max_output;
	}
	else if(pid->LocSum < 0.0f)
	{
		pid->LocSum = 0.0f;
	}

	// 计算PID各项 (添加时间补偿到微分项)
	proportional = pid->kp * pid->ek0;
	integral = pid->ki * pid->LocSum;
	differential = pid->kd * (pid->ek0 - pid->ek1) / ts_normalized;
	output = proportional + integral + differential;

	if(output < 0.0f)
	{
		output = 0.0f;
	}
	if(output > max_output)
	{
		output = max_output;
	}

	// 频率爬升斜率限制：默认约 4Hz/s（ts=500ms 时每周期约 2Hz）
	max_step = 4.0f * ts_normalized;
	delta = output - pid->last_output;
	if(delta > max_step)
	{
		output = pid->last_output + max_step;
	}
	else if(delta < -max_step)
	{
		output = pid->last_output - max_step;
	}

	if(output < 0.0f)
	{
		output = 0.0f;
	}
	if(output > max_output)
	{
		output = max_output;
	}

	pid->ek2 = pid->ek1;
	pid->ek1 = pid->ek0;
	pid->last_output = output;  // 改进5：记录上一次输出

	return (u16)(min_freq + output);
}
