#ifndef _COMM_ 
#define _COMM_

#include <stm32f1xx_hal.h>
#include "Sensors.h"
#include "SEGGER_RTT.h"

void can_init();

void can_fetch_reference();

void can_send_data();

#endif