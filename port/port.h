/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "sys.h"
#include "includes.h"

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include <inttypes.h>
#include "stm32f10x.h"

#define	INLINE                      
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )     __set_PRIMASK(1)
#define EXIT_CRITICAL_SECTION( )      __set_PRIMASK(0)

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#define USART_DMA

#define RTU_PROTORCAL_VERSION		1

#define RTU_MEM(x) (x-RTU_MEM_BASE)

#if (RTU_PROTORCAL_VERSION == 1)
enum RTU_MOTOR_ADDR
{
/////////////motor work parameters////////
	ADDR_MOTOR_GET_FWD=0,
	ADDR_MOTOR_SET_RUN,
	ADDR_MOTOR_SET_VOL,	
	
  ADDR_SET_TEMP,
	ADDR_NTC_TEMP,
///////////////////////////////////////	
	ADDR_DOOR,
	ADDR_LOCK,
	ADDR_SET_LOCK,
	ADDR_SET_DC2,
	ADDR_SET_DC3,
	
	ADDR_SET_AC1,
	ADDR_SET_AC2,
	ADDR_SET_AC3,
	ADDR_SET_FEQ,
	ADDR_HREG_TOTAL,

	RTU_MEM_BASE= 0x1000,
};
#endif
typedef struct
{
	unsigned short* data;
}RTU_MemDef;
extern RTU_MemDef usRegHoldingBuf[ADDR_HREG_TOTAL];




#endif
