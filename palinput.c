/*
 * palinput.c
 *
 *  Created on: 25 июля 2020 г.
 *      Author: andru
 */

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "palinput.h"

#define IN1		PAL_LINE(GPIOA, GPIOA_IN1)
#define IN2		PAL_LINE(GPIOB, GPIOB_IN2)
#define IN3		PAL_LINE(GPIOB, GPIOB_IN3)
#define IN4		PAL_LINE(GPIOB, GPIOB_IN4)

volatile uint8_t input_line[LINE_NUM];

static void input1_cb(void *arg);
static void input2_cb(void *arg);
static void input3_cb(void *arg);
static void input4_cb(void *arg);

static volatile uint16_t input_mask;
static const io_line_event_t iolines[LINE_NUM] = {
  { IN1,  input1_cb },
  { IN2,  input2_cb },
  { IN3,  input3_cb },
  { IN4,  input4_cb },
};

static void input1_cb(void *arg) {
  (void)arg;
  chSysLockFromISR();
  input_mask |= MASK(0);
  chSysUnlockFromISR();
}

static void input2_cb(void *arg) {
  (void)arg;
  chSysLockFromISR();
  input_mask |= MASK(1);
  chSysUnlockFromISR();
}

static void input3_cb(void *arg) {
  (void)arg;
  chSysLockFromISR();
  input_mask |= MASK(2);
  chSysUnlockFromISR();
}

static void input4_cb(void *arg) {
  (void)arg;
  chSysLockFromISR();
  input_mask |= MASK(3);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waReadLines, 192);
static THD_FUNCTION(readLines, arg) {
  (void)arg;

  uint8_t lines[LINE_NUM];
  chRegSetThreadName("readLines");

  for (uint8_t i=0; i<LINE_NUM; i++) {
	  lines[i] = (input_line[i] ? LINE_OFF : LINE_ON);
  }
  systime_t next = chVTGetSystemTimeX();

  while (true) {
	next += TIME_MS2I(LINE_DELAY);
    thd_state |= THD_LINE;
	if (input_mask > 0) {
	  for (uint8_t i=0; i<LINE_NUM; i++) {
		if (input_mask & (1 << i)) {
		  uint8_t line = palReadLine(iolines[i].line) == PAL_HIGH;
		  lines[i] <<= 1;
		  lines[i] += line;
		  if (lines[i] == 0 || lines[i] == 0xFF) {
			input_line[i] = lines[i] == LINE_ON;
			chEvtBroadcastFlags(&event_src, MASK(i));
			input_mask &= ~MASK(i);
		  }
		}
	  }
	}
	chThdSleepUntil(next);
  } // while
}

void input_init(void) {
  /* Enabling events.*/
  for (uint8_t i=0; i<LINE_NUM; i++) {
	palSetLineMode(iolines[i].line, PAL_MODE_INPUT);
	palEnableLineEvent(iolines[i].line, PAL_EVENT_MODE_BOTH_EDGES);
	palSetLineCallback(iolines[i].line, iolines[i].cb, NULL);
	input_line[i] = palReadLine(iolines[i].line) == LINE_ON;
  }
  // Creates lines read thread.
  (void) chThdCreateStatic(waReadLines, sizeof(waReadLines), NORMALPRIO+1, readLines, NULL);
}
