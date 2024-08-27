/*
 * init.h
 *
 *  Created on: May 30, 2024
 *      Author: thede
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

void SystemClock_Config(void);
void configureTimerForRunTimeStats(void);
void RTOS_Stats_Timer_Init(void);
void BankA_Init(void);
void BankC_Init(void);
void LEDinit(void);
void TIM2_Init(void);
void DMA_WithADC_Init(void);
void ADC_Init(void);
void USART_Init(void);

#endif /* INC_INIT_H_ */
