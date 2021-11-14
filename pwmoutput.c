/*
 * pwmoutput.c
 *
 *  Created on: 20.07.2020
 *      Author: andru
 */
#include "ch.h"
#include "hal.h"

#include "pwmoutput.h"

#define PWMDriver	PWMD2
#define PWM1		PAL_LINE(GPIOA, GPIOA_PWM1)
#define PWM2		PAL_LINE(GPIOA, GPIOA_PWM2)
#define PWM3		PAL_LINE(GPIOA, GPIOA_PWM3)
#define PWM4		PAL_LINE(GPIOA, GPIOA_PWM4)

static const PWMConfig pwmcfg = {
	2000000, 	/* 2Mhz PWM clock frequency */
	100, 		/* PWM period 50us */
	NULL,  		/* No callback */
	/* Channel 1-4 enabled */
	{
		{PWM_OUTPUT_ACTIVE_LOW, NULL},
		{PWM_OUTPUT_ACTIVE_LOW, NULL},
		{PWM_OUTPUT_ACTIVE_LOW, NULL},
		{PWM_OUTPUT_ACTIVE_LOW, NULL},
	},
	0,
	0
};

void pwm_init(void) {
	pwmStart(&PWMDriver, &pwmcfg);

	/* Enables PWM output (of TIM2, channel 0-3) */
	palSetLineMode(PWM1, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetLineMode(PWM2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetLineMode(PWM3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetLineMode(PWM4, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
}

void pwmchan_off(uint8_t channel) {
	if (channel > 3) return;
	pwmDisableChannel(&PWMDriver, PWMCH(channel));
}

void pwmchan_on(uint8_t channel, uint8_t percent) {
	if (channel > 3) return;

	if (percent == 0)
		pwmDisableChannel(&PWMDriver, PWMCH(channel));
	if (percent > 0 && percent <= 100)
		pwmEnableChannel(&PWMDriver, PWMCH(channel),
						 PWM_PERCENTAGE_TO_WIDTH(&PWMDriver, percent * 100));
}

uint8_t is_pwmchan(uint8_t channel) {
	uint8_t state = 0;
	if (channel > 3) return state;
	chSysLock();
	state = pwmIsChannelEnabledI(&PWMDriver, PWMCH(channel));
	chSysUnlock();
	return state;
}
