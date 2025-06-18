/*
 * ADC_DMA_PWM.h
 *
 *  Created on: 17 θών. 2025 γ.
 *      Author: mrbru
 */

#ifndef ADC_DMA_PWM_H_
#define ADC_DMA_PWM_H_

#include "stm32f4xx.h"

#define DATA_SIZE 2

void init_ADC();
void init_DMA(uint16_t* memory);
void init_PWM();
void start(uint16_t* memory);
void set_PWM_adv(uint16_t data);
void set_PWM_basic(uint16_t data);

#endif /* ADC_DMA_PWM_H_ */
