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
 * File: $Id: portevent.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
//#include <stdlib.h>
#include <ch.h>


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"

/* ----------------------- Variables ----------------------------------------*/
static msg_t bufferQueue;
static mailbox_t xQueueHdl;


/* ----------------------- Start implementation -----------------------------*/
#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0

BOOL
xMBPortEventInit( void )
{
  chMBObjectInit(&xQueueHdl, &bufferQueue, 1);
  return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
  BOOL            bStatus = TRUE;

  if (bMBPortIsWithinException () == TRUE) {
    if (chMBPostI (&xQueueHdl, (msg_t) eEvent) != MSG_OK)
      bStatus = FALSE;
  } else {
    if (chMBPostTimeout(&xQueueHdl, (msg_t) eEvent, TIME_INFINITE) != MSG_OK)
      bStatus = FALSE;
  }

  return bStatus;
}

BOOL
xMBPortEventGet( eMBEventType * peEvent )
{
    BOOL            xEventHappened = FALSE;

    if(chMBFetchTimeout(&xQueueHdl, (msg_t *) peEvent, TIME_MS2I(50)) == MSG_OK)
      xEventHappened = TRUE;

    return xEventHappened;
}

#endif

void
vMBPortEventClose( void )
{
  chMBReset (&xQueueHdl);
}
