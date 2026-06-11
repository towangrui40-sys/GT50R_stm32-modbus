/*
************************************************************************************************
主要的包含文件

文 件: INCLUDES.C ucos包含文件
作 者: Jean J. Labrosse
************************************************************************************************
*/

#ifndef __INCLUDES_H__
#define __INCLUDES_H__
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>

#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"

#include "stm32f10x.h"

#include "sys.h"

#include "gpio.h"
#include "delay.h"
#include "myiic.h"
#include "adc.h"
#include "dac.h"
#include "usart.h"

#include "lIS302DL.h"


typedef struct {
	short fwd;
	short set_run;
	short set_vol;
}MOTOR_TYPEDEF;

typedef struct
{
	MOTOR_TYPEDEF mc_para;
	short set_temp;
	short ntc_temp;
	short door;
	short lock;
	short set_lock;
	short set_dc2;
	short set_dc3;
	short set_ac1;
	short set_ac2;
	short set_ac3;
	short set_feq;
}SysDataDef;


#endif































