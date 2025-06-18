/*
 * ADC_DMA_PWM.c
 *
 *  Created on: 17 θών. 2025 γ.
 *      Author: mrbru
 */

#include "ADC_DMA_PWM.h"

void init_ADC(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//setting analog inputs on IN0(PA0) and IN1(PA1)
	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP1_0;
	ADC1->SQR3 |= ((0<<0) | (1<<5));
	ADC1->SQR1 |= (1<<20);

	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void init_DMA(uint16_t* memory){
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE;
	DMA2_Stream0->M0AR = (uint32_t)memory;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->NDTR = DATA_SIZE;
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);

	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void init_PWM(){

	//PA6 -> TIM3_CH1 basic TIM
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOA->AFR[0] |= (2  << (6*4));
	TIM3->ARR = 4100-1;
	TIM3->PSC = 100-1;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 1);
	TIM3->CR1 |= TIM_CR1_CEN;

	//PA7 -> TIM1_CH1N advanced TIM
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOA->AFR[0] |= (1 << (7*4));
	TIM1->ARR = 4100-1;
	TIM1->PSC = 100-1;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC1P;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->BDTR |= TIM_BDTR_MOE | TIM_BDTR_OSSR;
	TIM1->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0);
	TIM1->CR1 |= TIM_CR1_CEN;
}

void set_PWM_adv(uint16_t data){
	TIM1->CCR1 = data;
}

void set_PWM_basic(uint16_t data){
	TIM3->CCR1 = data;
}

void start(uint16_t* memory){
	init_ADC();
	init_DMA(memory);
	init_PWM();
	set_PWM_adv(2000);
	set_PWM_basic(2000);
}
