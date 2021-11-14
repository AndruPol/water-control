/*
 * config.c
 *
 *  Created on: 09 апр. 2019 г.
 *      Author: andru
*/

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "config.h"
#include "eeprom.h"
#include "crc8.h"
#include "modbus_slave.h"

#define DHT_READ		0	// DHT sensor read interval, S
#define DHT_TEMP		45	// DHT sensor temperature critical
#define DHT_HUM			80	// DHT sensor humidity critical
#define LIGHT_TIME		180	// default light time on move sensor
#define VALVE_TIME		5	// default valve 100% power time
#define VALVE_DELAY		1	// default valve power on delay, S
#define VALVE_POWER		100	// default valve power in continue
#define MOTOR_DELAY		10	// default motor power on delay, S
#define MOTOR_TIME		180	// default motor power on time, S
#define MOTOR_POWER		100	// default motor power
#define COUNTER_INC		10	// default increase counter on pulse

volatile backup_data_t* backup_regs = (volatile backup_data_t*)(BKP_BASE + 0x0004);

// set config default values
void defaultconfig(void) {
	config.magic = MAGIC;
	config.mb_addr = MB_ADDR;				// Modbus address
	config.mb_speed = MB_SPEED;				// Modbus speed
	config.valve_on = true;					// enable valve auto on
	config.cntinc[0] = COUNTER_INC;			// counter 0 step
	config.cntinc[1] = COUNTER_INC;			// counter 0 step
	config.input[0].type = INPUT_COUNTER1;	// water counter
	config.input[0].active = ACTIVE_HIGH;	// water counter active
	config.input[1].type = INPUT_COUNTER2;	// water counter
	config.input[1].active = ACTIVE_HIGH;	// water counter active
	config.input[2].type = INPUT_WSENSOR;	// water sensor
	config.input[2].active = ACTIVE_HIGH;	// water sensor active
	config.input[3].type = INPUT_WSENSOR;	// water sensor
	config.input[3].active = ACTIVE_HIGH;	// water sensor active
	config.pwm_out[0].type = PWM_OUTPUT_VALVE1;		// pwm output 1 type
	config.pwm_out[0].delay = VALVE_DELAY;			// delay, S before pwm on
	config.pwm_out[0].time = VALVE_TIME;			// pwm output 1 100% time, S
	config.pwm_out[0].power = VALVE_POWER;			// pwm output 1 percent
	config.pwm_out[1].type = PWM_OUTPUT_VALVE2;		// pwm output 2 type
	config.pwm_out[1].delay = VALVE_DELAY;			// delay, S before pwm on
	config.pwm_out[1].time = VALVE_TIME;			// pwm output 2 100% time, S
	config.pwm_out[1].power = VALVE_POWER;			// pwm output 2 percent
}

// read config from eeprom
bool readconfig(void) {
	uint8_t buf[sizeof(config_t) + 1];
    eeprom_error_t status = eepromRead(buf, sizeof(config_t) + 1);
    if (status != EEPROM_OK)
        return FALSE;

    uint8_t crc = CRC8(buf, sizeof(config_t));
    if (buf[sizeof(config_t)] != crc)
        return FALSE;

    memcpy((uint8_t *) &config, buf, sizeof(config_t));
    if (config.magic != MAGIC)
    	return FALSE;

    return TRUE;
}

// write config values eeprom
bool writeconfig(void) {
    uint16_t magic;
    uint8_t buf[sizeof(config_t) + 1];

    uint8_t crc = CRC8((uint8_t *) &config, sizeof(config_t));
    memcpy(buf, (uint8_t *) &config, sizeof(config_t));
    buf[sizeof(config_t)] = crc;
    eeprom_error_t status = eepromWrite(buf, sizeof(config_t) + 1);
    if (status != EEPROM_OK) {
        return FALSE;
    }

    // delay between EEPROM write & read > 4mS
    chThdSleepMilliseconds(5);
    eepromRead((uint8_t *) &magic, sizeof(magic));

    if (magic != MAGIC) {
        return FALSE;
    }
	return TRUE;
}

// read water counter value from backup registers
bool readcounter(uint8_t id, counter_t* wcnt) {
	if (backup_regs->magic.data != MAGIC) {
		return FALSE;
	}
	switch (id) {
	case 0:
		wcnt->data.reg[0] = backup_regs->water1_reg0.data;
		wcnt->data.reg[1] = backup_regs->water1_reg1.data;
		break;
	case 1:
		wcnt->data.reg[0] = backup_regs->water2_reg0.data;
		wcnt->data.reg[1] = backup_regs->water2_reg1.data;
		break;
	default:
		return FALSE;
	}
	return TRUE;
}

// write water counter value to backup regs
bool writecounter(uint8_t id, counter_t* wcnt) {
	backup_regs->magic.data = MAGIC;
	switch (id) {
	case 0:
		backup_regs->water1_reg0.data = wcnt->data.reg[0];
		backup_regs->water1_reg1.data = wcnt->data.reg[1];
		break;
	case 1:
		backup_regs->water2_reg0.data = wcnt->data.reg[0];
		backup_regs->water2_reg1.data = wcnt->data.reg[1];
		break;
	default:
		return FALSE;
	}
	if (backup_regs->magic.data != MAGIC) {
		return FALSE;
	}
	return TRUE;
}
