/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
//#include "init.h"
//#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <complex.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	char command[50];
	uint8_t len;
} command_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define CLK_FREQUENCY 32000000 // system clock
#define SAMPLING_FREQUENCY 2048 // sampling frequency
#define NUM_SAMPLES 2048 // number of samples
#define DELTA_F (float)SAMPLING_FREQUENCY/(float)NUM_SAMPLES // frequency resolution
#define BAUD_RATE 460800 // baud rate for USART
#define GPIOC_PIN_MASK 0x07FF // mask for pins PC0 to PC10

extern volatile unsigned long ulHighFrequencyTimerTicks; // used for task runtime stats
extern q15_t digitalSampleArray[NUM_SAMPLES]; // array to hold the samples from ADC
extern q15_t FFTBuffer[NUM_SAMPLES * 2]; // array to hold values of FFT
extern float maxFrequency ; // to hold maxFrequency for printing
extern float desiredFrequency; // to hold current desired frequency
extern uint16_t dcOffset; // to hold offset for printing
extern arm_rfft_instance_q15 S; // FFT structure
extern uint8_t running; // flag to determine "on/off"
extern char command[50]; // to hold global command
extern uint8_t commandIndex; // for indexing into global command
extern char currentNote[50]; // for the current note to tune to
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
