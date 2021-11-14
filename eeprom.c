/*
 * eeprom.c
 *
 *  Created on: 11/08/2016
 *      Author: andru
 */

#include "ch.h"
#include "hal.h"

#include "hal_eeprom.h"
#include "hal_ee24xx.h"
#include "eeprom.h"
#include "eeprom_conf.h"

#define EEPROM_ADDR   	0x50
#define EEPROM_DRIVER	I2CD1
#define EEPROM_SCL		PAL_LINE(GPIOB, GPIOB_SCL)
#define EEPROM_SDA		PAL_LINE(GPIOB, GPIOB_SDA)

/* buffer for I2C read/write data */
static uint8_t eeprom_writebuf[EEPROM_PAGE_SIZE+2];
static I2CEepromFileStream efile;

/* I2C2 */
static const I2CConfig EEPROMConfig = {
    OPMODE_I2C,
    100000,
	STD_DUTY_CYCLE,
};

static I2CEepromFileConfig efilecfg = {
  0,
  0,
  EEPROM_SIZE,
  EEPROM_PAGE_SIZE,
  TIME_MS2I(EEPROM_WRITE_TIME_MS),
  &EEPROM_DRIVER,
  EEPROM_ADDR,
  eeprom_writebuf,
};

// init i2c driver for eeprom
void eepromInit(void) {
	if (EEPROM_DRIVER.state < I2C_READY) {
		i2cStart(&EEPROM_DRIVER, &EEPROMConfig);

		// I2C1 remap PB6, PB7 -> PB8, PB9
		AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;

		/* tune pins for I2C2*/
		palSetLineMode(EEPROM_SCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		palSetLineMode(EEPROM_SDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	}
}

void eepromStop(void) {
	i2cStop(&EEPROM_DRIVER);
	palSetLineMode(EEPROM_SCL, PAL_MODE_INPUT);
	palSetLineMode(EEPROM_SDA, PAL_MODE_INPUT);
}

eeprom_error_t eepromRead(uint8_t *buf, uint16_t len) {
	EepromFileStream *iefs;
	eeprom_error_t status = EEPROM_OK;

	efilecfg.barrier_low  = EEPROM_AREA_START;
	efilecfg.barrier_hi   = EEPROM_AREA_SIZE(len);
	iefs = I2CEepromFileOpen(&efile, &efilecfg, EepromFindDevice(EEPROM_DEVICE));
	fileStreamSeek(iefs, 0);
	if (0 != fileStreamGetPosition(iefs))
	    status = EEPROM_ERR_SEEK;

	if (status == EEPROM_OK && fileStreamRead(iefs, buf, len) != len) {
		status = EEPROM_ERR_READ;
	}

	fileStreamClose(iefs);

	return status;
}

eeprom_error_t eepromWrite(uint8_t *buf, uint16_t len) {
	EepromFileStream *oefs;
	eeprom_error_t status = EEPROM_OK;

	efilecfg.barrier_low  = EEPROM_AREA_START;
	efilecfg.barrier_hi   = EEPROM_AREA_SIZE(len);
	oefs = I2CEepromFileOpen(&efile, &efilecfg, EepromFindDevice(EEPROM_DEVICE));
	fileStreamSeek(oefs, 0);
	if (0 != fileStreamGetPosition(oefs))
	   status = EEPROM_ERR_SEEK;

	/* write */
	if (status == EEPROM_OK && fileStreamWrite(oefs, buf, len) != len)
	   status = EEPROM_ERR_WRITE;

	/* check */
	if (status == EEPROM_OK && fileStreamGetPosition(oefs) != len)
	   return EEPROM_ERR_WRITE;

	fileStreamClose(oefs);

	return status;
}
