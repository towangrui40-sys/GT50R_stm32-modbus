/*************************************************************************/
/*	freemodbus ÓÃ»§×Ô¶¨Òå½Ó¿Úº¯Êý	
	eMBRegInputCB 
	eMBRegHoldingCB 
	eMBRegCoilsCB	
	eMBRegDiscreteCB									*/
/************************************************************************/

#include "mb.h"
#include "stm32f10x.h"

int modbus_flag;

extern void
xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,
                UCHAR ucValue );

extern UCHAR
xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits );

//¹¦ÄÜÂë 04(¶ÁÖ»¶Á¼Ä´æÆ÷) ¼Ä´æÆ÷3x
/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )	 	
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}


//¹¦ÄÜÂë 03 06 16(¶ÁÐ´¼Ä´æÆ÷) ¼Ä´æÆ÷ 4x
#define REG_HOLDING_START RTU_MEM_BASE
#define REG_HOLDING_NREGS ADDR_HREG_TOTAL

static USHORT   usRegHoldingStart = REG_HOLDING_START;
//static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];


//u16 usRegHoldingBuf[REG_HOLDING_NREGS]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,9,10};	//²âÊÔÓÃÀý


RTU_MemDef usRegHoldingBuf[REG_HOLDING_NREGS];
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
   	eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    /* it already plus one in modbus function method. */
    usAddress--;
	
	if( ( usAddress >= REG_HOLDING_START )
        && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
	{
	  	iRegIndex = ( int )( usAddress - usRegHoldingStart );
		switch(eMode)
		{
			case MB_REG_READ:
				while( usNRegs > 0 )
				{
					*pucRegBuffer++ =
							( unsigned char )(usRegHoldingBuf[iRegIndex].data[0]>> 8); 
						//( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
					*pucRegBuffer++ =
							( unsigned char )(usRegHoldingBuf[iRegIndex].data[0]& 0xFF ); 
					   //( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
					iRegIndex++;
					usNRegs--;
				}
			break;
			case MB_REG_WRITE:	
				while( usNRegs > 0 )
				{

					usRegHoldingBuf[iRegIndex].data[0] = *pucRegBuffer++ << 8;
					usRegHoldingBuf[iRegIndex].data[0] |=	*pucRegBuffer++;	
				
					
					iRegIndex++;
					usNRegs--;
				}
			break;
		}
	}
	else 
		eStatus = MB_ENOREG;
	return eStatus;		
}


//¹¦ÄÜÂë 01 05 15(¶ÁÐ´ÏßÈ¦) ¼Ä´æÆ÷ 0x
#define REG_COILS_START 1
#define REG_COILS_SIZE 16

static UCHAR   ucRegCoilsBuf[REG_COILS_SIZE];

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    //´íÎó×´Ì¬
	eMBErrorCode eStatus = MB_ENOERR;
	//¼Ä´æÆ÷¸öÊý
	int16_t iNCoils = ( int16_t )usNCoils;
	//¼Ä´æÆ÷Æ«ÒÆÁ¿
	int16_t usBitOffset;
	
	//¼ì²é¼Ä´æÆ÷ÊÇ·ñÔÚÖ¸¶¨·¶Î§ÄÚ
	if( ( (int16_t)usAddress >= REG_COILS_START ) &&
	( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
	{
		//¼ÆËã¼Ä´æÆ÷Æ«ÒÆÁ¿
		usBitOffset = ( int16_t )( usAddress - REG_COILS_START );
		switch ( eMode )
		{
		//¶Á²Ù×÷
		case MB_REG_READ:
			while( iNCoils > 0 )
			{
				*pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
				( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );
				iNCoils -= 8;
				usBitOffset += 8;
			}
			break;
		
		//Ð´²Ù×÷
		case MB_REG_WRITE:
			while( iNCoils > 0 )
			{
				xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
				( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),
				*pucRegBuffer++ );
				iNCoils -= 8;
			}
			break;
		}
	
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}


//¹¦ÄÜÂë 02(¶ÁÖ»¶ÁÏßÈ¦) ¼Ä´æÆ÷ 1x
#define REG_DISCRETE_START 1
#define REG_DISCRETE_SIZE 16
static UCHAR   ucRegDiscreteBuf[REG_DISCRETE_SIZE];

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
   //´íÎó×´Ì¬
	eMBErrorCode eStatus = MB_ENOERR;
	//²Ù×÷¼Ä´æÆ÷¸öÊý
	int16_t iNDiscrete = ( int16_t )usNDiscrete;
	//Æ«ÒÆÁ¿
	uint16_t usBitOffset;
	
	//ÅÐ¶Ï¼Ä´æÆ÷Ê±ºòÔÙÖÆ¶¨·¶Î§ÄÚ
	if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
	( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
	{
	//»ñµÃÆ«ÒÆÁ¿
		usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );
	
		while( iNDiscrete > 0 )
		{
			*pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
			( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
			iNDiscrete -= 8;
			usBitOffset += 8;
		}
	
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}




