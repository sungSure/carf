#ifndef __WIRELESS_H_
#define __WIRELESS_H_

#include "stm32f1xx_hal.h"

extern uint8_t Rx_Buff[3], Tx_Buff[10];

void Wireless_Control(void);
#endif 
