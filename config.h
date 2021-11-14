/*
 * config.h
 *
 *  Created on: 09 апр. 2019 г.
 *      Author: andru
*/

#ifndef CONFIG_H_
#define CONFIG_H_

typedef struct _counter counter_t;
struct _counter {
	union {
		uint32_t value;		// water counter data
		uint16_t reg[2];	// backup registers map data
	} data;
};

typedef struct backup_reg backup_reg_t;
struct backup_reg {
	uint16_t data;
	uint16_t na;
};

typedef struct backup_data backup_data_t;
struct backup_data {
	backup_reg_t	magic;			// magic value
	backup_reg_t	water1_reg0;	// water counter 1 low data
	backup_reg_t	water1_reg1;	// water counter 1 high data
	backup_reg_t	water2_reg0;
	backup_reg_t	water2_reg1;
	backup_reg_t	config;			// configuration data
	backup_reg_t	state;			// state data
};

#ifdef __cplusplus
extern "C" {
#endif

void defaultconfig(void);
bool readconfig(void);
bool writeconfig(void);
bool readcounter(uint8_t id, counter_t* wcnt);
bool writecounter(uint8_t id, counter_t* wcnt);

extern volatile backup_data_t* backup_regs;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
