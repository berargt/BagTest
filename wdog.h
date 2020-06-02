#ifndef WDOG_H
#define WDOG_H

#include <avr/wdt.h>

void wdog_start();

void wdog_reset();

// watchdog reset handler
//isr(wdt_vect);

#endif // WDOG_H
