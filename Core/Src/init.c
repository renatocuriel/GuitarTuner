/*
 * init.c
 *
 *  Created on: May 30, 2024
 *      Author: thede
 */

#include "init.h"
#include "main.h"

void BankA_Init(void)
{
	RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN; // enable bank A clock

    // Configure PA5 as output (LED)
    GPIOA->MODER &= ~GPIO_MODER_MODE5_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;

	// Set PA1 to alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODE1;
	GPIOA->MODER |= GPIO_MODER_MODE1_1;

	// Set alternate function type for PA1 (AF1 for TIM2_CH2)
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk; //clear bits
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL1_0; //set bits

	// High speed, no pull up or pull down
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;
}

void BankC_Init(void)
{
    // Enable GPIOC peripheral clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Init LEDs
    LEDinit();

    // Configure PC13 as input (Button)
    GPIOC->MODER &= ~GPIO_MODER_MODE13;

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTI13 line for PC13
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13_Msk;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    // Enable EXTI13 interrupt
    EXTI->IMR1 |= EXTI_IMR1_IM13;
    EXTI->RTSR1 |= EXTI_RTSR1_RT13;

    // Configure NVIC for EXTI15_10
    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void LEDinit(void)
{
    // Set PC0 to PC10 as outputs
    GPIOC->MODER &= ~(
        GPIO_MODER_MODE0_Msk |
        GPIO_MODER_MODE1_Msk |
        GPIO_MODER_MODE2_Msk |
        GPIO_MODER_MODE3_Msk |
        GPIO_MODER_MODE4_Msk |
        GPIO_MODER_MODE5_Msk |
        GPIO_MODER_MODE6_Msk |
        GPIO_MODER_MODE7_Msk |
        GPIO_MODER_MODE8_Msk |
        GPIO_MODER_MODE9_Msk |
        GPIO_MODER_MODE10_Msk
    );

    GPIOC->MODER |= (
        GPIO_MODER_MODE0_0 |
        GPIO_MODER_MODE1_0 |
        GPIO_MODER_MODE2_0 |
        GPIO_MODER_MODE3_0 |
        GPIO_MODER_MODE4_0 |
        GPIO_MODER_MODE5_0 |
        GPIO_MODER_MODE6_0 |
        GPIO_MODER_MODE7_0 |
        GPIO_MODER_MODE8_0 |
        GPIO_MODER_MODE9_0 |
        GPIO_MODER_MODE10_0
    );

    // Set output type to push-pull (default state)
    GPIOC->OTYPER &= ~(
        GPIO_OTYPER_OT0 |
        GPIO_OTYPER_OT1 |
        GPIO_OTYPER_OT2 |
        GPIO_OTYPER_OT3 |
        GPIO_OTYPER_OT4 |
        GPIO_OTYPER_OT5 |
        GPIO_OTYPER_OT6 |
        GPIO_OTYPER_OT7 |
        GPIO_OTYPER_OT8 |
        GPIO_OTYPER_OT9 |
        GPIO_OTYPER_OT10
    );

    // Set output speed to high
    GPIOC->OSPEEDR |= (
        GPIO_OSPEEDR_OSPEED0_1 |
        GPIO_OSPEEDR_OSPEED1_1 |
        GPIO_OSPEEDR_OSPEED2_1 |
        GPIO_OSPEEDR_OSPEED3_1 |
        GPIO_OSPEEDR_OSPEED4_1 |
        GPIO_OSPEEDR_OSPEED5_1 |
        GPIO_OSPEEDR_OSPEED6_1 |
        GPIO_OSPEEDR_OSPEED7_1 |
        GPIO_OSPEEDR_OSPEED8_1 |
        GPIO_OSPEEDR_OSPEED9_1 |
        GPIO_OSPEEDR_OSPEED10_1
    );

    // Set no pull-up/pull-down
    GPIOC->PUPDR &= ~(
        GPIO_PUPDR_PUPD0_Msk |
        GPIO_PUPDR_PUPD1_Msk |
        GPIO_PUPDR_PUPD2_Msk |
        GPIO_PUPDR_PUPD3_Msk |
        GPIO_PUPDR_PUPD4_Msk |
        GPIO_PUPDR_PUPD5_Msk |
        GPIO_PUPDR_PUPD6_Msk |
        GPIO_PUPDR_PUPD7_Msk |
        GPIO_PUPDR_PUPD8_Msk |
        GPIO_PUPDR_PUPD9_Msk |
        GPIO_PUPDR_PUPD10_Msk
    );
}

void TIM2_Init(void)
{
   RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable the TIM2 peripheral clock
   TIM2->CNT = 0; // Ensure count starts at 0
   TIM2->ARR = (CLK_FREQUENCY/SAMPLING_FREQUENCY) - 1; // auto reload for sampling freq
   TIM2->CCR2 = ((CLK_FREQUENCY/SAMPLING_FREQUENCY) - 1) / 2; // set 50% duty cycle for PWM mode (ARR/2)

   // Config channel 2 to PWM Mode 1, output
   TIM2->CCMR1 &= TIM_CCMR1_CC2S; // Channel 2 configured as output
   TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // MODE 1; 0110

   // Enable compare mode for Channel 2
   TIM2->CCER |= TIM_CCER_CC2E;

   // Set upcounting, Enable TIM2
   TIM2->CR1 &= ~TIM_CR1_DIR;
   TIM2->CR1 |= TIM_CR1_CEN; // TIMER ENABLE
}

void DMA_WithADC_Init(void)
{
	//Enable DMA1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//Set up peripheral to memory mode,
	//sizes of transfer (16 bits), automatic increments on memory address,
	//circular mode, direction (read from peripheral, ADC1),
	//and interrupts on complete transfer
	DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; //turn mem2mem mode off
	DMA1_Channel1->CCR &= ~DMA_CCR_DIR; //set dir = 0
	DMA1_Channel1->CCR &= ~DMA_CCR_PINC; //turn off peripheral increment
	DMA1_Channel1->CCR |= (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC |
							DMA_CCR_CIRC | DMA_CCR_TCIE);

	//Set number of data to transfer
	DMA1_Channel1->CNDTR = NUM_SAMPLES;

	//Set addresses to read/write to/from
	DMA1_Channel1->CPAR = (uint32_t) &(ADC1->DR); //source (DIR = 0)
	DMA1_Channel1->CMAR = (uint32_t) digitalSampleArray; //destination (DIR = 0)

	//Select DMA Channel
	DMA1_CSELR->CSELR &= ~(DMA_CSELR_C1S); //set C1S (where ADC1 is connected) to 0000

	//Initialize NVIC to be FreeRTOS safe
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // set NVIC Priority Grouping
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // set interrupt priorities
	NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable interrupt
	DMA1_Channel1->CCR |= DMA_CCR_EN; //enable DMA1 Channel 1
}

void ADC_Init(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //turn on clock for ADC
	ADC123_COMMON->CCR = ((ADC123_COMMON->CCR & ~(ADC_CCR_CKMODE)) | ADC_CCR_CKMODE_0); // set ADC to be clocked at PCLK/2
	ADC1->CR &= ~ADC_CR_ADSTART; //make sure start bit is cleared to be able to run config

	//power up ADC and voltage regulator
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	ADC1->CR |= ADC_CR_ADVREGEN;

	//ADC Voltage Regulator delay
	for (uint16_t i = 0; i < 1000; i++)
	  for (uint16_t j = 0; j < 100; j++);

	//Calibrate ADC
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); //ensure ADC is not enabled, single ended calibration
	ADC1->CR |= ADC_CR_ADCAL; //start calibration
	while (ADC1->CR & ADC_CR_ADCAL); //wait for calibration to finish
	ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5; //PA0 is channel 5. Set low for single ended.

	//ENABLE ADC
	ADC1->ISR |= ADC_ISR_ADRDY; //clear adrdy flag
	ADC1->CR |= ADC_CR_ADEN; //enable ADC1
	while (!(ADC1->ISR & ADC_ISR_ADRDY)); //wait for ADC1 to be ready to start conversion
	ADC1->ISR |= ADC_ISR_ADRDY; //clear adrdy flag

	//configure ADC
	ADC1->CFGR &= ~ADC_CFGR_CONT; //single conversion
	ADC1->CFGR &= ~(ADC_CFGR_RES); //12 bit resolution
	ADC1->CFGR |= ADC_CFGR_EXTEN_0; //enable hardware trigger on rising edge
	ADC1->CFGR |= ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0; //enable hardware trigger source (TIM2_CH2)
	ADC1->CFGR |= ADC_CFGR_DMACFG; //set DMA circular mode
	ADC1->CFGR |= ADC_CFGR_DMAEN; //enable DMA mode
	ADC1->CFGR |= ADC_CFGR_OVRMOD; // enable discard on overrun (most recent sample always in ADC->DR)
	ADC1->SMPR1 &= ~(0x3) << ADC_SMPR1_SMP5_Pos; //set sample rate to 2.5 clock cycles

	//clear 1st conversion and sequence length bits
	//set 1st conversion to channel 5
	ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1_Msk | ADC_SQR1_L_Msk))
					| (5 << ADC_SQR1_SQ1_Pos);

	//configure GPIO pin PA0 (channel 5)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable GPIOA clock
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0)) | (7 << GPIO_AFRL_AFSEL0_Pos);
	GPIOA->MODER |= GPIO_MODER_MODE0; //alternate func
	GPIOA->ASCR |= GPIO_ASCR_ASC0;
}

void USART_Init(void)
{
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);    // mask AF selection
	GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) |             // select USART2 (AF7)
					 (7 << GPIO_AFRL_AFSEL3_Pos));             // for PA2 and PA3
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);     // enable alternate function
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);  // for PA2 and PA3

	// Configure USART2 connected to the debugger virtual COM port
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;            // enable USART by turning on system clock
	USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);     // set data to 8 bits
	USART2->BRR = CLK_FREQUENCY / BAUD_RATE;           // baudrate
	USART2->CR1 |= USART_CR1_UE;                       // enable USART
	USART2->CR1 |= (USART_CR1_TE);      // enable transmit for USART

	// Set interrupt priority
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // set NVIC Priority Grouping
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // set interrupt priorities
    NVIC_EnableIRQ(USART2_IRQn);
}

/* Configure Timer to interrupt 100 kHz (100 times every Tick) */
void RTOS_Stats_Timer_Init(void)
{
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM5EN);	// turn on TIM5
	TIM5->DIER |= (TIM_DIER_UIE);			// enable interrupts
	TIM5->SR  &= ~(TIM_SR_UIF);				// clear interrupt flag
	TIM5->ARR = CLK_FREQUENCY/100000 - 1;
	TIM5->CR1 |= TIM_CR1_CEN;			    // start timer

	// enable interrupts
	NVIC->ISER[0] = (1 << (TIM5_IRQn & 0x1F));
}

/* Built in functions for using FreeRTOS runtime stats need to be defined*/
void configureTimerForRunTimeStats(void)
{
    ulHighFrequencyTimerTicks = 0;
    RTOS_Stats_Timer_Init();
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
