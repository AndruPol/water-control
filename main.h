/*
 * main.h
 *
 *  Created on: 23.07.2020
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#define	FIRMWARE		100		// firmware version
#define MAGIC			0xAE68	// magic word

#define COUNTER_NUM		2
#define PWMOUT_NUM		2
#define OUTPUT_NUM		(PWMOUT_NUM)

/*
 * 	event flag masks
 *	EVT_INPUT_IN1 - IN10 defined by palinput.h
*/
#define EVT_EVENT_TEST		(1 << 16)
#define EVT_MB_COIL_WRITE	(1 << 17)
#define EVT_MB_HOLD_WRITE	(1 << 18)

#define MASK(n)				(1 << n)

typedef enum {
	INPUT_UNDEF = 0,
	INPUT_WSENSOR,			// water sensor
	INPUT_COUNTER1,			// water counter 1
	INPUT_COUNTER2,			// water counter 2
} input_type_t;

typedef enum {
	ACTIVE_LOW,
	ACTIVE_HIGH,
	ACTIVE_ANY,
} input_active_t;

typedef enum {
	OUTPUT_UNDEF = 0,
	PWM_OUTPUT_VALVE1,		// water valve 1
	PWM_OUTPUT_VALVE2,		// water valve 2
} output_type_t;

typedef struct _input_config_t input_config_t;
struct _input_config_t {
	input_type_t type;		//
	input_active_t active;	//
};

typedef struct _outline_t outline_t;
struct _outline_t {
	output_type_t output;	// defined output
	uint8_t state;			// output line state
	ioline_t line;			// PAL LINE
};

typedef struct _outline_config_t outline_config_t;
struct _outline_config_t {
	output_type_t	type;
	uint8_t time;
};

typedef struct _pwm_config_t pwm_config_t;
struct _pwm_config_t {
	output_type_t type;
	uint8_t delay;
	uint8_t time;
	uint8_t power;
};

typedef enum {
	OUTPUT_LINE,
	OUTPUT_PWM,
} timer_type_t;

typedef struct _output_timer_t output_timer_t;
struct _output_timer_t {
	uint8_t out;		// output id
	timer_type_t type;	// line or pwm
	bool active;		// active flag
	uint8_t time;		// timer time, S
	uint8_t value;		// set value
};

#ifdef __cplusplus
extern "C" {
#endif

#include "palinput.h"
#include "config.h"

typedef struct config config_t;
struct config {
	uint16_t magic;						// magic world
	uint8_t mb_addr;					// Modbus address
	uint8_t mb_speed;					// Modbus port speed
	uint8_t valve_on;					// enable valves on if water sensors off
	uint8_t cntinc[COUNTER_NUM];		// counter step array
	input_config_t input[LINE_NUM];		// input lines
	pwm_config_t pwm_out[PWMOUT_NUM];	// pwm output lines
};

#define THD_GOOD	0b111111
#define THD_INIT	0

typedef enum {
	THD_MAIN	= (1 << 0),
	THD_MODBUS  = (1 << 1),
	THD_TIMER 	= (1 << 2),
	THD_LINE 	= (1 << 3),
	THD_EVENT	= (1 << 4),
	THD_COUNTER = (1 << 5),
} thdcheck_t;

extern volatile thdcheck_t thd_state;
extern config_t config;
extern counter_t counter[COUNTER_NUM];
extern event_source_t event_src;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
