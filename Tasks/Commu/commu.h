#ifndef _COMM_
#define _COMM_

#include <stm32f1xx_hal.h>

#include "SEGGER_RTT.h"
#include "Sensors.h"

void can_init();

void can_fetch_reference();

void can_send_data();

void can_receive_data(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);

#endif