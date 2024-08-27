/*
  ******************************************************************************
  * @file           : main.c
  * author			: Diego Renato Curiel
  * date created	: 5/29/2024
  * last modified	:
  ******************************************************************************
  * Guitar Tuner description
  * Simple six string, standard guitar tuner. On board peripherals used include the
  * DMA, ADC, USART2, TIM2, TIM5, and a ton of GPIO pins for LED outputs.
  * The program is written utlizing FreeRTOS in tickless mode, to enable a
  * lower power consumption when the user isn't actively tuning. The way to
  * enter sleep and exit sleep is through the on board blue button. The low
  * power mode utilized is sleep.
  ******************************************************************************
*/

#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "init.h"
#include "utils.h"

SemaphoreHandle_t dmaTransferCompleteSemaphore, fftCompleteSemaphore, commandSemaphore; // semaphores for tasks
TaskHandle_t FFTHandler, printUSARTHandler, commandHandler; // create handler names
QueueHandle_t CommandQueue; // queue for passing commands safely from USART interrupt

void DMA1_Channel1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM5_IRQHandler(void);
void USART2_IRQHandler(void);

void FFTtask(void *argument);
void printUSARTtask(void *argument);
void commandTask(void *argument);

volatile unsigned long ulHighFrequencyTimerTicks; // used for task runtime stats
q15_t digitalSampleArray[NUM_SAMPLES] = {0}; // array to hold the samples from ADC
q15_t FFTBuffer[NUM_SAMPLES * 2] = {0}; // array to hold values of FFT
float maxFrequency = 0.0f; // to hold maxFrequency for printing
float desiredFrequency = 0.0f; // to hold current desired frequency
uint16_t dcOffset = 0; // to hold offset for printing
arm_rfft_instance_q15 S; // FFT structure
uint8_t running = 0; // flag to determine "on/off"
uint8_t high = 0; // user input to determine tuning below 200Hz
char command[50] = {0}; // to hold global command
uint8_t commandIndex = 0; // for indexing into global command
char currentNote[50] = {0}; // for the current note to tune to

int main(void)
{
	// INITIALIZE ALL PERIPHERALS
	HAL_Init();
	SystemClock_Config();
	configureTimerForRunTimeStats();
	BankA_Init();
	BankC_Init();
	TIM2_Init();
	USART_Init();

	USART_ESC_Code("[2J"); //clear terminal
	USART_ESC_Code("[H"); //back to top left
	USART_ESC_Code("[?25l"); //hide cursor
	USART_ESC_Code("[37m"); //white text

	//Initialize FFT configuration
	arm_rfft_init_q15(&S, NUM_SAMPLES, 0, 1);

	//Ensure all LEDs are off
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	GPIOC->ODR &= ~(GPIOC_PIN_MASK);

    // Create semaphores
    dmaTransferCompleteSemaphore = xSemaphoreCreateBinary();
    fftCompleteSemaphore = xSemaphoreCreateBinary();
    commandSemaphore = xSemaphoreCreateBinary();

    if (dmaTransferCompleteSemaphore == NULL || fftCompleteSemaphore == NULL || commandSemaphore == NULL) {while(1);}

    // Create Command Queue
    CommandQueue = xQueueCreate(2, sizeof(command_t));
    if (CommandQueue == NULL) { while(1); }

	// Create tasks
	BaseType_t retVal;
	retVal = xTaskCreate(FFTtask, "FFTtask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &FFTHandler);
	if (retVal != pdPASS) { while(1); }	// check if task creation failed

	retVal = xTaskCreate(printUSARTtask, "printUSARTtask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &printUSARTHandler);
	if (retVal != pdPASS) { while(1); }	// check if task creation failed

	retVal = xTaskCreate(commandTask, "commandTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &commandHandler);
	if (retVal != pdPASS) { while(1); } // check if task creation failed

	// Notify user that board is starting in sleep mode
	USART_Print("Sleeping...");

	// START ADC/DMA LAST TO ENSURE ALL RTOS CONFIG COMPLETES BEFORE DMA INTERRUPT HITS
	DMA_WithADC_Init();
	ADC_Init();

	// Start scheduler
	vTaskStartScheduler();

	printf("Shouldn't be here!\n");
	while (1) {}

}

/* Define Tasks --------------------------------------------------------------*/
void FFTtask(void *argument)
{
	for(;;)
	{
		if (xSemaphoreTake(dmaTransferCompleteSemaphore, portMAX_DELAY) == pdTRUE) // take semaphore
		{
			// compute the FFT!
			computeFFT();

			// give semaphore to calculate bins task
			xSemaphoreGive(fftCompleteSemaphore);
		}
	}
}

void printUSARTtask(void *argument)
{
	for(;;)
	{
		if (xSemaphoreTake(fftCompleteSemaphore, portMAX_DELAY) == pdTRUE)
		{
			if (running)
			{
				// update LEDS
				desiredFrequency = getFrequencyFromNote((const char*) currentNote);
				updateLEDs(maxFrequency, desiredFrequency);

				// restart ADC for next conversion/FFT!
				ADC1->CR |= ADC_CR_ADSTART;
			}
		}
	}
}

void commandTask(void *argument)
{
	for(;;)
	{
		if (xSemaphoreTake(commandSemaphore, portMAX_DELAY) == pdTRUE)
		{
			command_t cmd;
			if (xQueueReceive(CommandQueue, &cmd, 0) == pdTRUE)
			{
				// parse and execute user command!
				if(cmd.len < 50)
				{
					memcpy(currentNote, cmd.command, cmd.len);
					currentNote[cmd.len] = '\0';
				}
				char buffer[100] = {0};
				snprintf(buffer, sizeof(buffer), "Currently tuning to note %s\r\n", currentNote);
				USART_Print(buffer);
				USART_Print("$: ");
			}
		}
	}
}

/* INTERRUPT HANDLERS-------------------------------------------------------------*/

/* Interrupt Handler for DMA */
void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF1) // check transfer complete intr flag
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		ADC1->CR |= ADC_CR_ADSTP; // stop conversions
		DMA1->IFCR |= DMA_IFCR_CTCIF1; // clear transfer complete intr flag

		// Give FFT task semaphore
		xSemaphoreGiveFromISR(dmaTransferCompleteSemaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(&xHigherPriorityTaskWoken); // yield to next highest priority task (fft)
	}
}

/* Interrupt Handler for Button Press */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF13) // check button interrupt flag
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        EXTI->PR1 = EXTI_PR1_PIF13; // Clear interrupt flag
        GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LED
        GPIOC->ODR &= ~(GPIOC_PIN_MASK); // Turn off scale LEDs

        running = !running; // toggle global running flag (turn on or off)

        if (running) // if just turned on, start conversion and turn on USART RX interrupts
        {
			USART_ESC_Code("[2J"); //clear terminal
			USART_ESC_Code("[H"); //back to top left
        	printHeader();
        	USART_Print("$: ");
        	ADC1->CR |= ADC_CR_ADSTART;
        	USART2->CR1 |= (USART_CR1_RE);      // enable receive for USART2
        	USART2->CR1 |= USART_CR1_RXNEIE;                   // enable RXNE interrupt on USART2
        	USART2->ISR &= ~(USART_ISR_RXNE);                  // clear interrupt flag
        }
        else // if just turned off, turn off USART RX interrupts so board can only be woken by button
        {
			USART_ESC_Code("[2J"); //clear terminal
			USART_ESC_Code("[H"); //back to top left
			USART_Print("Sleeping...");
        	USART2->CR1 &= ~USART_CR1_RE; 							// disable receive for USART2
        	USART2->CR1 &= ~USART_CR1_RXNEIE;                   // disable RXNE interrupt on USART2
        	USART2->ISR &= ~(USART_ISR_RXNE);                  // clear interrupt flag
        	commandIndex = 0; // reset global index when turned off
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE)
    {
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    	if (USART2->RDR == '\r')
    	{
			USART_Print("\r\n");
			command_t cmd_t;
			memcpy(cmd_t.command, command, commandIndex);
			cmd_t.command[commandIndex] = '\0';
			cmd_t.len = commandIndex;

			xSemaphoreGiveFromISR(commandSemaphore, &xHigherPriorityTaskWoken);
			xQueueSendFromISR(CommandQueue, &cmd_t, &xHigherPriorityTaskWoken);

			commandIndex = 0; //reset message index
		}
		else if (USART2->RDR == 0x7F)// backspace
		{
			USART2->TDR = USART2->RDR; // echo
			if (commandIndex > 0)
			{
				commandIndex--;
			}
		}
    	else
    	{
    		USART2->TDR = USART2->RDR; // copy received char to transmit buffer to echo
    		command[commandIndex] = USART2->RDR; //put char into string
    		commandIndex++; // increment current index
    	}

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Timer 5 is used to collect runtime stats for FreeRTOS tasks*/
void TIM5_IRQHandler(void)
{
	TIM5->SR &= ~(TIM_SR_UIF);
	ulHighFrequencyTimerTicks++;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
