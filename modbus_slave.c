/*
 * modbus_slave.c
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "modbus_slave.h"

#include "main.h"
#include "pwmoutput.h"

#if 0
static const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFFF7E8;
static const uint8_t UniqProcessorIdLen = 12;
#endif

/*------------------------Slave mode use these variables----------------------*/
//Slave mode:DiscreteInputs variables
USHORT   usSDiscInStart                               = S_DISCRETE_INPUT_START;
#if S_DISCRETE_INPUT_NDISCRETES%8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8]  ;
#endif
//Slave mode:Coils variables
USHORT   usSCoilStart                                 = S_COIL_START;
#if S_COIL_NCOILS%8
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif
//Slave mode:InputRegister variables
USHORT   usSRegInStart                                = S_REG_INPUT_START;
SHORT   usSRegInBuf[S_REG_INPUT_NREGS]                ;
//Slave mode:HoldingRegister variables
USHORT   usSRegHoldStart                              = S_REG_HOLDING_START;
SHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]            ;

/**
 * Modbus slave input register callback function.
 *
 * @param pucRegBuffer input register buffer
 * @param usAddress input register address
 * @param usNRegs input register number
 *
 * @return result
 */
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    SHORT *         pusRegInputBuf;
    USHORT          REG_INPUT_START;
    USHORT          REG_INPUT_NREGS;
    USHORT          usRegInStart;

    pusRegInputBuf = usSRegInBuf;
    REG_INPUT_START = S_REG_INPUT_START;
    REG_INPUT_NREGS = S_REG_INPUT_NREGS;
    usRegInStart = usSRegInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_INPUT_START)
            && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = usAddress - usRegInStart;
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] & 0xFF);
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

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    SHORT *         pusRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

    pusRegHoldingBuf = usSRegHoldBuf;
    REG_HOLDING_START = S_REG_HOLDING_START;
    REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
    usRegHoldStart = usSRegHoldStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_HOLDING_START)
            && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldStart;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
			chEvtBroadcastFlags(&event_src, EVT_MB_HOLD_WRITE);
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucCoilBuf;
    USHORT          COIL_START;
    USHORT          COIL_NCOILS;
    USHORT          usCoilStart;
    iNReg =  usNCoils / 8 + 1;

    pucCoilBuf = ucSCoilBuf;
    COIL_START = S_COIL_START;
    COIL_NCOILS = S_COIL_NCOILS;
    usCoilStart = usSCoilStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= COIL_START ) &&
        ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],
                        iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
                        *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                        *pucRegBuffer++);
            }
			chEvtBroadcastFlags(&event_src, EVT_MB_COIL_WRITE);
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          DISCRETE_INPUT_START;
    USHORT          DISCRETE_INPUT_NDISCRETES;
    USHORT          usDiscreteInputStart;
    iNReg =  usNDiscrete / 8 + 1;

    pucDiscreteInputBuf = ucSDiscInBuf;
    DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
    DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
    usDiscreteInputStart = usSDiscInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= DISCRETE_INPUT_START)
            && (usAddress + usNDiscrete    <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES))
    {
        iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],
                    iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

bool initModbus (void)
{
  eMBErrorCode eStatus;
  uint32_t bitrate = 0;

  switch (config.mb_speed) {
  case MB_BITRATE_1200:
          bitrate = 1200;
          break;
  case MB_BITRATE_2400:
          bitrate = 2400;
          break;
  case MB_BITRATE_4800:
          bitrate = 4800;
          break;
  case MB_BITRATE_9600:
          bitrate = 9600;
          break;
  case MB_BITRATE_19200:
          bitrate = 19200;
          break;
  case MB_BITRATE_38400:
          bitrate = 38400;
          break;
  case MB_BITRATE_57600:
          bitrate = 57600;
          break;
  case MB_BITRATE_115200:
      bitrate = 57600;
      break;
  }

  if (!bitrate) return FALSE;

  eStatus = eMBInit(MB_MODE, config.mb_addr, MB_PORT, bitrate, MB_PARITY);
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  eStatus = eMBEnable();
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  pxMBPortCBTimerExpired();

  return TRUE;
}

/*
 * The MODBUS main thread
 */

static THD_WORKING_AREA(waThdModbus, MB_WASIZE);
static THD_FUNCTION(thdModbus, arg)
{
  (void)arg;

  chRegSetThreadName("MODBUS");

  while (initModbus() != TRUE) {
    chThdSleepMilliseconds(1000);

    if (chThdShouldTerminateX())
      goto cleanAndExit;
  }

  chThdSleepMilliseconds(10);

  do {
    eMBPoll();
    thd_state |= THD_MODBUS;
  } while (!chThdShouldTerminateX());

cleanAndExit:
  eMBDisable();
  eMBClose();
}

void createModbusThd(void) {
  chThdCreateStatic(waThdModbus, sizeof(waThdModbus), MB_PRIO, thdModbus, NULL);
}

void writeSysTick(void) {
	usSRegInBuf[MB_IN_SYSTICK]++;
}

UCHAR getDiscBit(USHORT regAddr) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          usDiscreteInputStart;

  if (regAddr < S_DISCRETE_INPUT_NDISCRETES) {
	    pucDiscreteInputBuf = ucSDiscInBuf;
	    usDiscreteInputStart = usSDiscInStart;

	    if (regAddr + 1    <= S_DISCRETE_INPUT_START + S_DISCRETE_INPUT_NDISCRETES)
	    {
	        iRegIndex = (USHORT) (regAddr - usDiscreteInputStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usDiscreteInputStart) % 8;
	        return xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex], iRegBitIndex, 1);
	    }
  }
  return 0xFF;
}

void setDiscBit(USHORT regAddr, UCHAR ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          usDiscreteInputStart;

  if (regAddr < S_DISCRETE_INPUT_NDISCRETES) {
	    pucDiscreteInputBuf = ucSDiscInBuf;
	    usDiscreteInputStart = usSDiscInStart;

	    if (regAddr + 1    <= S_DISCRETE_INPUT_START + S_DISCRETE_INPUT_NDISCRETES)
	    {
	        iRegIndex = (USHORT) (regAddr - usDiscreteInputStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usDiscreteInputStart) % 8;
	        xMBUtilSetBits(&pucDiscreteInputBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
  }
}

UCHAR getCoilBit(USHORT regAddr) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucCoilBuf;
    USHORT          usCoilStart;

    if (regAddr < S_COIL_NCOILS) {
    	pucCoilBuf = ucSCoilBuf;
    	usCoilStart = usSCoilStart;

	    if (regAddr + 1    <= S_COIL_START + S_COIL_NCOILS)
	    {
	        iRegIndex = (USHORT) (regAddr - usCoilStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usCoilStart) % 8;
            return xMBUtilGetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, 1);
	    }
    }
    return 0xFF;
}

void setCoilBit(USHORT regAddr, UCHAR ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucCoilBuf;
    USHORT          usCoilStart;

    if (regAddr < S_COIL_NCOILS) {
    	pucCoilBuf = ucSCoilBuf;
    	usCoilStart = usSCoilStart;

	    if (regAddr + 1    <= S_COIL_START + S_COIL_NCOILS)
	    {
	        iRegIndex = (USHORT) (regAddr - usCoilStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usCoilStart) % 8;
            xMBUtilSetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
    }
}

USHORT readInputReg(USHORT regAddr) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  return usSRegInBuf[regAddr];
  }
  return 0;
}

UCHAR readInputRegLo(USHORT regAddr) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  return (UCHAR) usSRegInBuf[regAddr] & 0x00FF;
  }
  return 0;
}

UCHAR readInputRegHi(USHORT regAddr) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  return (UCHAR) ((USHORT) usSRegInBuf[regAddr] >> 8);
  }
  return 0;
}

void writeInputReg(USHORT regAddr, USHORT regValue) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  usSRegInBuf[regAddr] = regValue;
  }
}

void writeInputRegLo(USHORT regAddr, UCHAR regValue) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  USHORT reg = usSRegInBuf[regAddr] & 0xFF00;
	  usSRegInBuf[regAddr] = reg | regValue;
  }
}

void writeInputRegHi(USHORT regAddr, UCHAR regValue) {
  if (regAddr < S_REG_INPUT_NREGS) {
	  UCHAR reg = usSRegInBuf[regAddr] & 0x00FF;
	  usSRegInBuf[regAddr] = (USHORT) (regValue << 8) | reg;
  }
}

ULONG readInputReg32(USHORT regAddr) {
  if (regAddr < (S_REG_INPUT_NREGS - 1)) {
	  return (ULONG) ((USHORT) usSRegInBuf[regAddr]) << 16 | (USHORT) usSRegInBuf[regAddr+1];
  }
  return 0;
}

void writeInputReg32(USHORT regAddr, ULONG value) {
  if (regAddr < (S_REG_INPUT_NREGS - 1)) {
	  usSRegInBuf[regAddr+1] = (USHORT) (value & 0x0000FFFF);
	  usSRegInBuf[regAddr] = (USHORT) (value >> 16);
  }
}

void writeInputRegFloat(USHORT regAddr, float regValue) {
  SHORT val;
  if (regAddr < S_REG_INPUT_NREGS) {
	  val = (SHORT)  (regValue > 0) ? (regValue + 0.5) : (regValue - 0.5);
	  usSRegInBuf[regAddr] = val;
  }
}

USHORT readHoldingReg(USHORT regAddr) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  return usSRegHoldBuf[regAddr];
  }
  return 0;
}

UCHAR readHoldingRegLo(USHORT regAddr) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  return (UCHAR) usSRegHoldBuf[regAddr] & 0x00FF;
  }
  return 0;
}

UCHAR readHoldingRegHi(USHORT regAddr) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  return (UCHAR) ((USHORT) usSRegHoldBuf[regAddr] >> 8);
  }
  return 0;
}

void writeHoldingReg(USHORT regAddr, USHORT regValue) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  usSRegHoldBuf[regAddr] = regValue;
  }
}

void writeHoldingRegLo(USHORT regAddr, UCHAR regValue) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  USHORT reg = usSRegHoldBuf[regAddr] & 0xFF00;
	  usSRegHoldBuf[regAddr] = reg | regValue;
  }
}

void writeHoldingRegHi(USHORT regAddr, UCHAR regValue) {
  if (regAddr < S_REG_HOLDING_NREGS) {
	  UCHAR reg = usSRegHoldBuf[regAddr] & 0x00FF;
	  usSRegHoldBuf[regAddr] = (USHORT) (regValue << 8) | reg;
  }
}

ULONG readHoldingReg32(USHORT regAddr) {
  if (regAddr < (S_REG_HOLDING_NREGS - 1)) {
	  return (ULONG) ((USHORT) usSRegHoldBuf[regAddr]) << 16 | (USHORT) usSRegHoldBuf[regAddr+1];
  }
  return 0;
}

void writeHoldingReg32(USHORT regAddr, ULONG value) {
  if (regAddr < (S_REG_HOLDING_NREGS - 1)) {
	  usSRegHoldBuf[regAddr+1] = (USHORT) (value & 0x0000FFFF);
	  usSRegHoldBuf[regAddr] = (USHORT) (value >> 16);
  }
}

void initModbusRegs(void) {
	writeInputRegLo(MB_IN_MBCONFIG, config.mb_addr);
	writeInputRegHi(MB_IN_MBCONFIG, config.mb_speed);
	writeInputReg32(MB_IN_CNT1LO, counter[0].data.value);
	writeInputReg32(MB_IN_CNT2LO, counter[1].data.value);
	setCoilBit(MB_CO_VALVE, config.valve_on);
	for (uint8_t i=0; i<LINE_NUM; i++) {
		writeHoldingRegLo(MB_HO_IN1 + i, config.input[i].type);
		writeHoldingRegHi(MB_HO_IN1 + i, config.input[i].active);
		setDiscBit(MB_DI_IN1 + i, input_line[i]);
		setCoilBit(MB_CO_IN1 + i, OFF);
	}
	for (uint8_t i=0; i<PWMOUT_NUM; i++) {
		uint8_t state = is_pwmchan(i);
		setDiscBit(MB_DI_PWM1 + i, state);
		setCoilBit(MB_CO_PWM1 + i, state);
	}
	writeInputRegLo(MB_IN_PWM1LO, config.pwm_out[0].type);
	writeInputRegHi(MB_IN_PWM1LO, config.pwm_out[0].power);
	writeInputRegLo(MB_IN_PWM1HI, config.pwm_out[0].delay);
	writeInputRegHi(MB_IN_PWM1HI, config.pwm_out[0].time);

	writeInputRegLo(MB_IN_PWM2LO, config.pwm_out[1].type);
	writeInputRegHi(MB_IN_PWM2LO, config.pwm_out[1].power);
	writeInputRegLo(MB_IN_PWM2HI, config.pwm_out[1].delay);
	writeInputRegHi(MB_IN_PWM2HI, config.pwm_out[1].time);

	writeInputRegLo(MB_IN_CNTINC, config.cntinc[0]);
	writeInputRegHi(MB_IN_CNTINC, config.cntinc[1]);

	for (uint8_t i=0; i<LINE_NUM; i++) {
		writeInputRegLo(MB_IN_IN1 + i, config.input[i].type);
		writeInputRegHi(MB_IN_IN1 + i, config.input[i].active);
	}
}
