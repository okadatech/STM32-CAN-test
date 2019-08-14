/*
 * microsectimer.h
 *
 *  Created on: 2018/01/18
 *      Author: ryouma
 */

#ifndef MICROSECTIMER_H_
#define MICROSECTIMER_H_



#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <string.h>


uint32_t getUs(void);
void delayUs(uint16_t micros);
#endif /* MICROSECTIMER_H_ */
