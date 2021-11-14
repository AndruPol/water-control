/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdint.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "main.h"

#include "config.h"
#include "modbus_slave.h"
#include "pwmoutput.h"
#include "sensors.h"
#include "palinput.h"
#include "eeprom.h"

#define EEPROM			TRUE
#define WDG				TRUE

#define TIMERINT		1000	// 1 sec timer interval
#define COUNTERINT		100		// 100ms counter interval
#define COUNTERTIME		5		// counter min increase time

#define LED				PAL_LINE(GPIOA, GPIOA_LED)
#define PWRIN			PAL_LINE(GPIOA, GPIOA_PWRIN)

#define OUT1			PAL_LINE(GPIOB, GPIOB_OUT1)
#define OUT2			PAL_LINE(GPIOA, GPIOA_OUT2)

#define LINEON(n)		palClearLine(n);
#define LINEOFF(n)		palSetLine(n);

config_t config;
volatile thdcheck_t thd_state;
event_source_t event_src;

counter_t counter[COUNTER_NUM];
static volatile uint8_t counter_timer[COUNTER_NUM];

static bool write_config = false;

#define QUEUE_LEN		4
static output_timer_t queue_buffer[OUTPUT_NUM][QUEUE_LEN];
output_timer_t *output_queue_free[OUTPUT_NUM][QUEUE_LEN];
output_timer_t *output_queue_fill[OUTPUT_NUM][QUEUE_LEN];
mailbox_t mb_output_queue_free[OUTPUT_NUM], mb_output_queue_fill[OUTPUT_NUM];

output_timer_t timer[OUTPUT_NUM];

static void setTimer(output_timer_t *t);
static void clearTimer(output_timer_t *t);
static output_timer_t fillTimer(uint8_t out, timer_type_t type, uint8_t time, uint8_t value);


/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 2000)).
 */
#if WDG
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(2000),
};
#endif

static void setPWMchannel(uint8_t channel, uint8_t value) {
	if (channel > 3 || value > 100) return;
	if (value > 0) {
		if (config.pwm_out[channel].type == PWM_OUTPUT_VALVE1 ||
			config.pwm_out[channel].type == PWM_OUTPUT_VALVE2) {
			if (!is_pwmchan(channel) && config.pwm_out[channel].time) {
				output_timer_t t = fillTimer(
										channel,
										OUTPUT_PWM,
										config.pwm_out[channel].time,
										config.pwm_out[channel].power
									);
				setTimer(&t);
				pwmchan_on(channel, 100);
				writeInputRegHi(MB_IN_PWM1LO + (2 * channel), 100);
			} else {
				pwmchan_on(channel, value);
				writeInputRegHi(MB_IN_PWM1LO + (2 * channel), value);
			}
			setDiscBit(MB_DI_PWM1 + channel, ON);
			setCoilBit(MB_CO_PWM1 + channel, ON);
		}
	} else {
		output_timer_t t = fillTimer(channel, OUTPUT_PWM, 0, 0);
		clearTimer(&t);
		pwmchan_off(channel);
		writeInputRegHi(MB_IN_PWM1LO + (2 * channel), 0);
		setDiscBit(MB_DI_PWM1 + channel, OFF);
		setCoilBit(MB_CO_PWM1 + channel, OFF);
	}
}

/*===========================================================================*/
static void input_handler(uint8_t line) {
	switch (config.input[line].type) {
	case INPUT_WSENSOR:
		if (input_line[line] == config.input[line].active) {
			// valves off
			for (uint8_t i=0; i<PWMOUT_NUM; i++) {
				if (config.pwm_out[i].type == PWM_OUTPUT_VALVE1 ||
					config.pwm_out[i].type == PWM_OUTPUT_VALVE2) {
					setPWMchannel(i, 0);
				}
			}
		} else {
			if (!config.valve_on) break;
			uint8_t fired = 0;
			for (uint8_t i=0; i<LINE_NUM; i++) {
				if (config.input[i].type == INPUT_WSENSOR &&
					input_line[i] == config.input[i].active)
					fired++;
			}
			if (fired > 0) break;
			// valves on
			for (uint8_t i=0; i<PWMOUT_NUM; i++) {
				if (config.pwm_out[i].type == PWM_OUTPUT_VALVE1 ||
					config.pwm_out[i].type == PWM_OUTPUT_VALVE2) {
					setPWMchannel(i, 100);
				}
			}
		}
		break;
	case INPUT_COUNTER1:
		if (input_line[line] == config.input[line].active &&
			counter_timer[0] == 0) {
			counter_timer[0] = COUNTERTIME;
			counter[0].data.value += config.cntinc[0];
			setCoilBit(MB_CO_CNT1_INC, ON);
			setDiscBit(MB_DI_BACKUP_ERR, OFF);
			writeInputReg32(MB_IN_CNT1LO, counter[0].data.value);
			if (!writecounter(0, &counter[0]))
			  setDiscBit(MB_DI_BACKUP_ERR, ON);
		}
		break;
	case INPUT_COUNTER2:
		if (input_line[line] == config.input[line].active &&
			counter_timer[1] == 0) {
			counter_timer[1] = COUNTERTIME;
			counter[1].data.value += config.cntinc[1];
			setCoilBit(MB_CO_CNT2_INC, ON);
			setDiscBit(MB_DI_BACKUP_ERR, OFF);
			writeInputReg32(MB_IN_CNT2LO, counter[1].data.value);
			if (!writecounter(1, &counter[1]))
			  setDiscBit(MB_DI_BACKUP_ERR, ON);
		}
		break;
	default:
		break;
	}
}

/*===========================================================================*/
static void modbus_coil_handler(void) {
    bool write = false;
    uint8_t coil;
    uint16_t reg;

    coil = getCoilBit(MB_CO_VALVE);
    if (coil != config.valve_on) {
    	config.valve_on = coil;
    	write = true;
    }

    if (getCoilBit(MB_CO_MBCONFIG)) {
        setCoilBit(MB_CO_MBCONFIG, OFF);
	    uint8_t addr = readHoldingRegLo(MB_HO_MBCONFIG);
	    uint8_t speed = readHoldingRegHi(MB_HO_MBCONFIG);
		if (addr > 0 && addr <= 32 && speed > 0 && speed <= MB_BITRATE_115200) {
			config.mb_addr = addr;
			config.mb_speed = speed;
			writeInputRegLo(MB_IN_MBCONFIG, config.mb_addr);
			writeInputRegHi(MB_IN_MBCONFIG, config.mb_speed);
			write = true;
		} else
			setCoilBit(MB_CO_CMD_ERR, ON);
    }

    if (getCoilBit(MB_CO_CNT1_SET)) {
        setCoilBit(MB_CO_CNT1_SET, OFF);
	    uint32_t cnt = readHoldingReg32(MB_HO_CNT1LO);
	    counter[0].data.value = cnt;
    	setDiscBit(MB_DI_BACKUP_ERR, OFF);
	    if (writecounter(0, &counter[0])) {
	    	writeInputReg32(MB_IN_CNT1LO, cnt);
	    } else
	    	setDiscBit(MB_DI_BACKUP_ERR, ON);
    }

    if (getCoilBit(MB_CO_CNT2_SET)) {
        setCoilBit(MB_CO_CNT2_SET, OFF);
	    uint32_t cnt = readHoldingReg32(MB_HO_CNT2LO);
	    counter[1].data.value = cnt;
    	setDiscBit(MB_DI_BACKUP_ERR, OFF);
	    if (writecounter(1, &counter[1])) {
	    	writeInputReg32(MB_IN_CNT2LO, cnt);
	    } else
	    	setDiscBit(MB_DI_BACKUP_ERR, ON);
    }

    for (uint8_t i=0; i<LINE_NUM; i++) {
    	if (getCoilBit(MB_CO_IN1 + i)) {
            setCoilBit(MB_CO_IN1 + i, OFF);
    	    uint8_t type = readHoldingRegLo(MB_HO_IN1 + i);
    	    uint8_t active = readHoldingRegHi(MB_HO_IN1 + i);
    		if (type <= INPUT_COUNTER2 && active <= ACTIVE_ANY) {
    			config.input[i].type = type;
    			config.input[i].active = active;
    			writeInputRegLo(MB_IN_IN1 + i, config.input[i].type);
    			writeInputRegHi(MB_IN_IN1 + i, config.input[i].active);
    			write = true;
    		} else
    			setCoilBit(MB_CO_CMD_ERR, ON);
    	}
    }

    for (uint8_t i=0; i<PWMOUT_NUM; i++) {
    	coil = getCoilBit(MB_CO_PWM1 + i);
		if (config.pwm_out[i].type != OUTPUT_UNDEF && coil != is_pwmchan(i)) {
			output_timer_t t = fillTimer(i, OUTPUT_PWM, 0, 0);
			clearTimer(&t);
			if (coil > 0) {
				setPWMchannel(i, config.pwm_out[i].power);
			} else {
				setPWMchannel(i, 0);
			}
		}
    }

    for (uint8_t i=0; i<PWMOUT_NUM; i++) {
    	if (getCoilBit(MB_CO_PWM1_SET + i)) {
			setCoilBit(MB_CO_PWM1_SET + i, OFF);
			uint8_t type = readHoldingRegLo(MB_HO_PWM1LO + (2*i));
			uint8_t power = readHoldingRegHi(MB_HO_PWM1LO + (2*i));
			uint8_t delay = readHoldingRegLo(MB_HO_PWM1HI + (2*i));
			uint8_t time = readHoldingRegHi(MB_HO_PWM1HI + (2*i));
			if (type == OUTPUT_UNDEF || (type >= PWM_OUTPUT_VALVE1)) {
				config.pwm_out[i].type = type;
				config.pwm_out[i].power = power;
				config.pwm_out[i].delay = delay;
				config.pwm_out[i].time = time;
				writeInputRegLo(MB_IN_PWM1LO + (2*i), config.pwm_out[i].type);
				writeInputRegHi(MB_IN_PWM1LO + (2*i), config.pwm_out[i].power);
				writeInputRegLo(MB_IN_PWM1HI + (2*i), config.pwm_out[i].delay);
				writeInputRegHi(MB_IN_PWM1HI + (2*i), config.pwm_out[i].time);
				write = true;
			} else
				setCoilBit(MB_CO_CMD_ERR, ON);
    	}
    }

    if (getCoilBit(MB_CO_CNT1INC)) {
        setCoilBit(MB_CO_CNT1INC, OFF);
	    reg = readHoldingReg(MB_HO_CNTINC);
	    if (reg <= 255) {
	    	config.cntinc[0] = (uint8_t) reg;
	    	writeInputRegLo(MB_IN_CNTINC, config.cntinc[0]);
	    	write = true;
	    } else
	    	setCoilBit(MB_CO_CMD_ERR, ON);
    }

    if (getCoilBit(MB_CO_CNT2INC)) {
        setCoilBit(MB_CO_CNT2INC, OFF);
	    reg = readHoldingReg(MB_HO_CNTINC);
	    if (reg <= 255) {
	    	config.cntinc[1] = (uint8_t) reg;
	    	writeInputRegHi(MB_IN_CNTINC, config.cntinc[1]);
	    	write = true;
	    } else
	    	setCoilBit(MB_CO_CMD_ERR, ON);
    }

    if (write) write_config = true;
}

/*===========================================================================*/
static void modbus_hold_handler(void) {
}


/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/
static void initTimerQueue(void) {
	for (uint8_t n=0; n<OUTPUT_NUM; n++) {
		chMBObjectInit(&mb_output_queue_free[n], (msg_t*) output_queue_free[n], QUEUE_LEN);
		chMBObjectInit(&mb_output_queue_fill[n], (msg_t*) output_queue_fill[n], QUEUE_LEN);

		// fill free buffers mailbox
		for (uint8_t i=0; i < QUEUE_LEN; i++)
			chMBPostTimeout(&mb_output_queue_free[n], (msg_t) &queue_buffer[n][i], TIME_IMMEDIATE);
	}

	for (uint8_t i=0; i<PWMOUT_NUM; i++) {
		timer[i].out = PWMCH(i);
		timer[i].type = OUTPUT_PWM;
		timer[i].active = false;
		timer[i].time = 0;
	}
}

static bool postTimerQueue(output_timer_t *t) {
    void *pbuf;

    uint8_t n = 0;
    n += t->out;

    if (chMBFetchTimeout(&mb_output_queue_free[n], (msg_t *) &pbuf, TIME_IMMEDIATE) != MSG_OK) {
    	return false;
    }
    memcpy(pbuf, t, sizeof(output_timer_t));
    if (chMBPostTimeout(&mb_output_queue_fill[n], (msg_t) pbuf, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }

    return true;
}

static bool fetchTimerQueue(output_timer_t *t) {
    void *pbuf;

    uint8_t n = 0;
    n += t->out;

    if (chMBFetchTimeout(&mb_output_queue_fill[n], (msg_t *) &pbuf, TIME_IMMEDIATE) != MSG_OK) {
     return false;
    }
    memcpy(&timer[n], pbuf, sizeof(output_timer_t));
    timer[n].active = true;
	if (chMBPostTimeout(&mb_output_queue_free[n], (msg_t) pbuf, TIME_IMMEDIATE) != MSG_OK) {
		return false;
    }

	return true;
}

static void clearTimerQueue(output_timer_t *t) {
    uint8_t n = 0;
    n += t->out;

    void *pbuf;
    while (chMBFetchTimeout(&mb_output_queue_fill[n], (msg_t *) &pbuf, TIME_IMMEDIATE) == MSG_OK) {
    	chMBPostTimeout(&mb_output_queue_free[n], (msg_t) pbuf, TIME_IMMEDIATE);
    }
}

static output_timer_t fillTimer(uint8_t out,
								timer_type_t type,
								uint8_t time,
								uint8_t value) {
	output_timer_t t = {
		.out = out,
		.type = type,
		.time = time,
		.value = value
	};
	return t;
}

static void setTimer(output_timer_t *t) {
    uint8_t n = 0;
    n += t->out;

    if (timer[n].active) {
    	postTimerQueue(t);
    } else {
    	timer[n].out = t->out;
    	timer[n].type = t->type;
    	timer[n].active = true;
    	timer[n].time = t->time;
    	timer[n].value = t->value;
    }
}

static void clearTimer(output_timer_t *t) {
    uint8_t n = 0;
    n += t->out;

    clearTimerQueue(t);

    timer[n].active = false;
	timer[n].time = 0;
	timer[n].value = 0;
}

static THD_WORKING_AREA(waTimerThread, 192);
static THD_FUNCTION(TimerThread, arg) {
	(void)arg;

	chRegSetThreadName("Timer");

	systime_t next = chVTGetSystemTimeX();

	while (true) {
		next += TIME_MS2I(TIMERINT);
    	thd_state |= THD_TIMER;

	    writeSysTick();

	    for (uint8_t i=0; i<OUTPUT_NUM; i++) {
	    	if (timer[i].active && timer[i].time == 0) {
	    		uint8_t out = timer[i].out;
	    		switch (timer[i].type) {
	    		case OUTPUT_PWM:
    				setPWMchannel(out, timer[i].value);
	    			break;
	    		default:
	    			break;
	    		}
	    		timer[i].active = false;
	    		output_timer_t t = fillTimer(timer[i].out, timer[i].type, 0, 0);
	    		fetchTimerQueue(&t);
	    	}
    		if (timer[i].active && timer[i].time > 0) timer[i].time--;
    		if (i % 2) {
    			writeInputRegHi(MB_IN_TIMER12 + (i / 2), timer[i].time);
    		} else {
    			writeInputRegLo(MB_IN_TIMER12 + (i / 2), timer[i].time);
    		}
	    }
		chThdSleepUntil(next);
	} // while
}

/*
 * events thread
*/
static THD_WORKING_AREA(waEventThread, 384);
static THD_FUNCTION(EventThread, arg) {
	(void)arg;
	event_listener_t event_el;

	chRegSetThreadName("Events");

	chEvtObjectInit(&event_src);
	chEvtRegisterMask(&event_src, &event_el, EVENT_MASK(0));

	while (TRUE) {
		chEvtWaitAny(EVENT_MASK(0));
   	    eventflags_t flags = chEvtGetAndClearFlags(&event_el);
   	    (void) flags;

   	    if (flags & EVT_EVENT_TEST)	// thread check
   	    	thd_state |= THD_EVENT;

   	    for (uint8_t i=0; i<LINE_NUM; i++) {
   	    	setDiscBit(MB_DI_IN1 + i, input_line[i]);
   	    	if (flags & MASK(i)) {
   	    		input_handler(i);
   	    	}
   	    }

   	    if (flags & EVT_MB_COIL_WRITE)
   	    	modbus_coil_handler();

   	    if (flags & EVT_MB_HOLD_WRITE)
   	    	modbus_hold_handler();

	} //while
}

static THD_WORKING_AREA(waCounterThread, 192);
static THD_FUNCTION(CounterThread, arg) {
	(void)arg;

	chRegSetThreadName("Counter");

	for (uint8_t i=0; i<COUNTER_NUM; i++)
		counter_timer[i] = 0;

	systime_t next = chVTGetSystemTimeX();
	while (true) {
		next += TIME_MS2I(COUNTERINT);
    	thd_state |= THD_COUNTER;
    	for (uint8_t i=0; i<COUNTER_NUM; i++) {
    		if (counter_timer[i] > 0) counter_timer[i]--;
    	}
		chThdSleepUntil(next);
	}
}

// called on kernel panic
void halt(void){
	port_disable();
	while(true) {
		palToggleLine(LED);
		chThdSleepMilliseconds(250);
	}
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  halInit();
  chSysInit();

  thd_state = THD_INIT;

#if WDG
  wdgStart(&WDGD1, &wdgcfg);
#endif

  // JTAG pins disable
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;


#if EEPROM
  // EEPROM init
  eepromInit();

  setDiscBit(MB_DI_EEPROM_ERR, OFF);
  // read or set default config from EEPROM
  if (!readconfig()) {
    defaultconfig();
    writeconfig();
    setDiscBit(MB_DI_EEPROM_ERR, ON);
  }
#else
  defaultconfig();
#endif

  setDiscBit(MB_DI_BACKUP_ERR, OFF);
  bool err = false;
  for (uint8_t i=0; i<COUNTER_NUM; i++) {
	if (!readcounter(i, &counter[i])) {
		writecounter(i, &counter[i]);
	    err = true;
	}
  }
  if (err) setDiscBit(MB_DI_BACKUP_ERR, ON);
  setCoilBit(MB_CO_RESTART, ON);
  writeInputReg(MB_IN_FIRMWARE, FIRMWARE);

  palSetLineMode(LED, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LED);

  // init PWM motor driver
  pwm_init();

  // Creates event manager thread.
  (void) chThdCreateStatic(waCounterThread, sizeof(waCounterThread), NORMALPRIO, CounterThread, NULL);

  // Creates event manager thread.
  (void) chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+2, EventThread, NULL);

  initTimerQueue();
  // Creates 1s timer thread.
  (void) chThdCreateStatic(waTimerThread, sizeof(waTimerThread), NORMALPRIO, TimerThread, NULL);

  sensors_init();

  // init PAL lines read thread.
  input_init();
  for (uint8_t i=0; i<LINE_NUM; i++) {
	setDiscBit(MB_DI_IN1 + i, input_line[i]);
	chEvtBroadcastFlags(&event_src, MASK(i));
  }

  if (config.valve_on) {
	  uint8_t fired = 0;
	  for (uint8_t i=0; i<LINE_NUM; i++) {
		  if (config.input[i].type == INPUT_WSENSOR &&
			  input_line[i] == config.input[i].active)
			  fired++;
	  }
	  if (fired == 0) {
		for (uint8_t i=0; i<PWMOUT_NUM; i++) {
			if (config.pwm_out[i].type == PWM_OUTPUT_VALVE1 ||
				config.pwm_out[i].type == PWM_OUTPUT_VALVE2) {
				setPWMchannel(i, 100);
			}
		}
	  }
  }

  // write Modbus registers from config
  initModbusRegs();

  // creates MODBUS polling thread
  createModbusThd();

  systime_t next = chVTGetSystemTimeX();

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
	next += TIME_MS2I(TIMERINT);

#if WDG
    // check process states & reset watchdog
    thd_state |= THD_MAIN;
    chEvtBroadcastFlags(&event_src, EVT_EVENT_TEST);
    if (thd_state == THD_GOOD) {
    	wdgReset(&WDGD1);
    	thd_state = THD_INIT;
    }
#endif

    palToggleLine(LED);

    if (write_config) {
    	write_config = false;
    	setDiscBit(MB_DI_EEPROM_ERR, OFF);
    	if (! writeconfig())
    	    setDiscBit(MB_DI_EEPROM_ERR, ON);
    }

    adc_error_t err = sensors_read();
    if (err == ADC_NO_ERROR) {
	    setDiscBit(MB_DI_TEMP_ERR, OFF);
	    writeInputRegFloat(MB_IN_INTTEMP, mcu_temp);
    } else {
	    setDiscBit(MB_DI_TEMP_ERR, ON);
    }

    chThdSleepUntil(next);
  }
}
