/*
 * pwmoutput.h
 *
 *  Created on: 20.07.2020
 *      Author: andru
 */

#ifndef PWMOUTPUT_H_
#define PWMOUTPUT_H_

#define PWM1				PAL_LINE(GPIOA, GPIOA_PWM1)
#define PWM2				PAL_LINE(GPIOA, GPIOA_PWM2)
#define PWM3				PAL_LINE(GPIOA, GPIOA_PWM3)
#define PWM4				PAL_LINE(GPIOA, GPIOA_PWM4)

#define PWMCH(n)			(3 - n)

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pwm_init(void);
void pwmchan_off(uint8_t channel);
void pwmchan_on(uint8_t channel, uint8_t percent);
uint8_t is_pwmchan(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* PWMOUTPUT_H_ */
