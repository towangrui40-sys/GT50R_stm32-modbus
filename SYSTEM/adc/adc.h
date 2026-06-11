#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"


#define TIMES 3
#define CHANNEL 4

void ADC1_Init(void);
u16 Get_Adc(u8 ch,float value,float fliter_num); 

#endif /* __ADC_H */

