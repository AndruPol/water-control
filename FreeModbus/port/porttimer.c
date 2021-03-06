/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: porttimer.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <ch.h>
#include <hal.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"

/* ----------------------- Defines ------------------------------------------*/

#define GPTDRIVER		GPTD1	// TIM1

static void timerHandler(GPTDriver *gptp);

/* ----------------------- Static variables ---------------------------------*/
/*
 * GPT2 configuration.
 */
static const GPTConfig gptcfg = {
  100000,    	/* 100kHz timer clock.*/
  timerHandler, /* Timer callback.*/
  0,
  0
};

static systime_t    timerout= 0;

/* ----------------------- Start implementation -----------------------------*/
static void timerHandler(GPTDriver *gptp)
{
  (void)gptp;

  chSysLockFromISR();
  vMBPortSetWithinException (TRUE) ;
#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0
  if (pxMBPortCBTimerExpired () == TRUE)
   	rescheduleJbus485FromIsr();
#endif
  vMBPortSetWithinException (FALSE) ;
  chSysUnlockFromISR();
}


#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0

BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  timerout = usTim1Timerout50us*((500*1000)/gptcfg.frequency);
  gptStart(&GPTDRIVER, &gptcfg);
  return TRUE;
}

void
vMBPortTimersEnable(  )
{
  if (bMBPortIsWithinException() == TRUE) {
    gptStopTimerI (&GPTDRIVER);
    gptStartOneShotI(&GPTDRIVER, timerout);
  } else { 
    gptStopTimer (&GPTDRIVER);
    gptStartOneShot(&GPTDRIVER, timerout);
  }
}

void
vMBPortTimersDisable(  )
{
  if (bMBPortIsWithinException() == TRUE)
    gptStopTimerI (&GPTDRIVER);
  else
    gptStopTimer (&GPTDRIVER);
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
  chThdSleepMicroseconds (usTimeOutMS);
}

#endif

void
vMBPortTimerClose( void )
{
   gptStop (&GPTDRIVER);
}
