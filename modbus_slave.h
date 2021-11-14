/*
 * modbus_slave.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include "port.h"

#define MB_ADDR		32				// slave address
#define MB_PORT		1				// not used now
#define MB_MODE		MB_RTU			// rtu mode only
#define MB_SPEED	5				// 19200
#define MB_PARITY	MB_PAR_NONE		// fixed

#define MB_PRIO		NORMALPRIO+1	// Modbus process priority
#define MB_WASIZE	384				// working area size

typedef enum {
    MB_BITRATE_1200 = 1,
    MB_BITRATE_2400,
    MB_BITRATE_4800,
    MB_BITRATE_9600,
    MB_BITRATE_19200,
    MB_BITRATE_38400,
    MB_BITRATE_57600,
    MB_BITRATE_115200,
} MB_BITRATE_MAP;

typedef enum {
	MB_PARITY_NONE = 0,
    MB_PARITY_EVEN,
    MB_PARITY_ODD,
} MB_PARITY_MAP;

typedef enum {
	OFF,
	ON,
} MB_ON_OFF;

typedef enum {
	MB_DI_EEPROM_ERR = 0,	// eeprom error
	MB_DI_BACKUP_ERR,		// backup registers error
	MB_DI_TEMP_ERR,			// internal temperature error
	MB_DI_IN1,				// input 1 state
	MB_DI_IN2,				// input 2 state
	MB_DI_IN3,				// input 3 state
	MB_DI_IN4,				// input 4 state
	MB_DI_PWM1,				// pwm output 1 state
	MB_DI_PWM2,				// pwm output 2 state
} MB_DISCRETE;

typedef enum {
	MB_CO_RESTART,			// restart flag
	MB_CO_CMD_ERR,			// Modbus command error flag
	MB_CO_VALVE,			// valve auto on flag
	MB_CO_MBCONFIG,			// Modbus address & speed set flag
	MB_CO_CNT1_INC,			// water counter 1 increased
	MB_CO_CNT2_INC,			// water counter 2 increased
	MB_CO_CNT1_SET,			// water counter 1 set flag
	MB_CO_CNT2_SET,			// water counter 2 set flag
	MB_CO_IN1,				// input 1 set type flag
	MB_CO_IN2,				// input 2 set type flag
	MB_CO_IN3,				// input 3 set type flag
	MB_CO_IN4,				// input 4 set type flag
	MB_CO_PWM1,				// pwm output 1 state
	MB_CO_PWM2,				// pwm output 2 state
	MB_CO_PWM1_SET,			// pwm output 1 set type flag
	MB_CO_PWM2_SET,			// pwm output 2 set type flag
	MB_CO_CNT1INC,			// counter 1 increase set
	MB_CO_CNT2INC,			// counter 2 increase set
} MB_COIL;

typedef enum {
	MB_IN_FIRMWARE,			// firmware version
	MB_IN_SYSTICK,			// dummy 1S counter
	MB_IN_INTTEMP,			// internal temperature
	MB_IN_CNT1LO,			// water counter 1 uint32_t lo register
	MB_IN_CNT1HI,			// water counter 1 uint32_t hi register
	MB_IN_CNT2LO,			// water counter 2 uint32_t lo register
	MB_IN_CNT2HI,			// water counter 2 uint32_t hi register
	MB_IN_MBCONFIG,			// Modbus address & speed
	MB_IN_PWM1LO,			// pwm output 1 (type & power)
	MB_IN_PWM1HI,			// pwm output 1 (delay & time)
	MB_IN_PWM2LO,			// pwm output 2 (type & power)
	MB_IN_PWM2HI,			// pwm output 2 (delay & time)
	MB_IN_TIMER12,			// timer 1 & 2 counter
	MB_IN_CNTINC,			// counter 1 & 2 increase step
	MB_IN_IN1,				// input 1 type & active
	MB_IN_IN2,				// input 2 type & active
	MB_IN_IN3,				// input 3 type & active
	MB_IN_IN4,				// input 4 type & active
} MB_INPUT;

typedef enum {
	MB_HO_MBCONFIG,
	MB_HO_CNT1LO,
	MB_HO_CNT1HI,
	MB_HO_CNT2LO,
	MB_HO_CNT2HI,
	MB_HO_IN1,
	MB_HO_IN2,
	MB_HO_IN3,
	MB_HO_IN4,
	MB_HO_PWM1LO,
	MB_HO_PWM1HI,
	MB_HO_PWM2LO,
	MB_HO_PWM2HI,
	MB_HO_CNTINC,
} MB_HOLDING;


/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   9
#define S_COIL_START                  0
#define S_COIL_NCOILS                 18
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             18
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           14
/* salve mode: holding register's all address */
#define S_HD_RESERVE                  0
#define S_HD_CPU_USAGE_MAJOR          1
#define S_HD_CPU_USAGE_MINOR          2
/* salve mode: input register's all address */
#define S_IN_RESERVE                  0
/* salve mode: coil's all address */
#define S_CO_RESERVE                  0
/* salve mode: discrete's all address */
#define S_DI_RESERVE                  0

//Slave mode:DiscreteInputs variables
extern USHORT   usSDiscInStart;
extern UCHAR    ucSDiscInBuf[];
//Slave mode:Coils variables
extern USHORT   usSCoilStart;
extern UCHAR    ucSCoilBuf[];
//Slave mode:InputRegister variables
extern USHORT   usSRegInStart;
extern SHORT   usSRegInBuf[];
//Slave mode:HoldingRegister variables
extern USHORT   usSRegHoldStart;
extern SHORT   usSRegHoldBuf[];

// create & start modbus poll process
void createModbusThd(void);
void writeSysTick(void);
void initModbusRegs(void);

// modbus helper functions
UCHAR getDiscBit(USHORT regAddr);
void setDiscBit(USHORT regAddr, UCHAR ucValue);
UCHAR getCoilBit(USHORT regAddr);
void setCoilBit(USHORT regAddr, UCHAR ucValue);
USHORT readInputReg(USHORT regAddr);
UCHAR readInputRegLo(USHORT regAddr);
UCHAR readInputRegHi(USHORT regAddr);
void writeInputReg(USHORT regAddr, USHORT regValue);
void writeInputRegLo(USHORT regAddr, UCHAR regValue);
void writeInputRegHi(USHORT regAddr, UCHAR regValue);
ULONG readInputReg32(USHORT regAddr);
void writeInputReg32(USHORT regAddr, ULONG value);
void writeInputRegFloat(USHORT regAddr, float regValue);
USHORT readHoldingReg(USHORT regAddr);
UCHAR readHoldingRegLo(USHORT regAddr);
UCHAR readHoldingRegHi(USHORT regAddr);
void writeHoldingReg(USHORT regAddr, USHORT regValue);
void writeHoldingRegLo(USHORT regAddr, UCHAR regValue);
void writeHoldingRegHi(USHORT regAddr, UCHAR regValue);
ULONG readHoldingReg32(USHORT regAddr);
void writeHoldingReg32(USHORT regAddr, ULONG value);

#endif /* MODBUS_SLAVE_H_ */
