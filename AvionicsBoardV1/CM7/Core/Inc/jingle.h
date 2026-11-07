#ifndef JINGLE_H
#define JINGLE_H

#include "main.h"

/* Initialize after TIM2 PWM setup. Notify functions may run in IRQ context. */
void Jingle_Init(void);
void Jingle_NotifySDResult(uint8_t success);
void Jingle_UpdateGPSLock(uint8_t locked);
/* Call from foreground. Patterns use elapsed ticks, without waiting. */
void Jingle_Service(void);

#endif
