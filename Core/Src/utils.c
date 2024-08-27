/*
 * utils.c
 *
 *  Created on: May 30, 2024
 *      Author: thede
 */

//#include "main.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

FrequencyNotePair sixStringStandard[] = {
    { NOTE_E2, "E2" },
    { NOTE_A, "A" },
    { NOTE_D, "D" },
    { NOTE_G, "G" },
    { NOTE_B, "B" },
    { NOTE_E3, "E3" }
};

float getFrequencyFromNote(const char *note)
{
	for(int i = 0; i < 6; i++)
	{
		if (strcasecmp(note, sixStringStandard[i].note) == 0)
		{
			return sixStringStandard[i].frequency;
		}
	}

	return 0.0;
}

void computeFFT(void)
{
	//Step 1: FFT
	arm_rfft_q15(&S, digitalSampleArray, FFTBuffer);

	//Step 2: Find Max Magnitude index
	uint16_t maxIndex = 0;
	uint16_t end = NUM_SAMPLES / 2; // only care about first half
	float maxMag = 0.0f; // to hold max magnitude for dB scaling
	for(uint16_t i = 1; i < end; i++) {
	  //calculate magnitude using real and imaginary parts
	  float rPart = (float) FFTBuffer[i * 2];
	  float iPart = (float) FFTBuffer[(i * 2) + 1];
	  float curMag = sqrtf(rPart * rPart + iPart * iPart);

	  //update maxMag, index if true
	  if (curMag > maxMag) {
		  maxMag = curMag;
		  maxIndex = i;
	  }
	}

	//Step 3: Find corresponding frequency
	maxFrequency = ((float) maxIndex) * DELTA_F;

	//Step 4: Digital signal conditioning
	//For frequencies below ~200 Hz, the fundamental frequency
	//registers with a lower magnitude than the harmonics of
	//the same frequency. These numbers were determined
	//experimentally.
	if(desiredFrequency < 120.0)
	{
		maxFrequency = maxFrequency / 2.0;
	}
//	else if (desiredFrequency < 200)
//	{
//		maxFrequency = maxFrequency / 2.0;
//	}

	// DEBUG
//	char debugBuf[100] = {0};
//	snprintf(debugBuf, sizeof(debugBuf), "maxF: %.2f desF: %.2f curNote: %s\r\n", maxFrequency, desiredFrequency, currentNote);
//	USART_Print(debugBuf);
}

void updateLEDs(float currentFrequency, float desiredFrequency)
{
    float difference = currentFrequency - desiredFrequency;
    uint32_t ledMask = 0x0;

    // Calculate which LED to turn on based on the frequency difference
    if (fabs(difference) <= 1) // Tight boundaries for green
    {
        ledMask = (1 << 5); // Turn on the green LED (PC5)
    }
    else if (difference > 0)
    {
        int ledIndex = 6 + ((int)(difference - 4) / 4);
        if (ledIndex > 10) ledIndex = 10;
        ledMask = (1 << ledIndex);
    }
    else
    {
        int ledIndex = 4 + ((int)(difference + 4) / 4);
        if (ledIndex < 0) ledIndex = 0;
        ledMask = (1 << ledIndex);
    }

    // Set the appropriate LED
    GPIOC->ODR = (GPIOC->ODR & ~GPIOC_PIN_MASK) | ledMask;
    if (ledMask == (1 <<5))
    {
    	vTaskDelay(pdMS_TO_TICKS(5000)); // delay on green for five seconds
    }
}

void printHeader(void)
{
	USART_Print("Welcome to Guitar Tuner v1.0!\r\n\tUsage:\r\n\t");
	USART_Print("This is a standard tuner for a six-string guitar: E2-A-D-G-B-E3\r\n\t");
	USART_Print("When prompted, please input the note you would like to tune to.\r\n\t");
	USART_Print("At any moment, simply input the new note you want to tune to. Happy tuning!\r\n\r\n");
}

void USART_MoveCursor(int row, int col)
{
	//init string
	char command[10] = {0};

	//make command
	snprintf(command, sizeof(command), "[%d;%dH", row, col);

	//USART_ESC_Code goes here
	USART_ESC_Code(command);
}

void USART_Print(const char* message)
{
   uint32_t i;
   for (i = 0; message[i] != 0; i++)
   {                // check for terminating NULL character
       while (!(USART2->ISR & USART_ISR_TXE));       // wait for transmit buffer to be empty
       USART2->TDR = message[i];                      // transmit character to USART
   }
}

void USART_ESC_Code(const char* message)
{
	while (!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = 0x1b; // code for 'ESC'
	USART_Print(message);
}

void printFrequency(void)
{
	  char freqString[15];
	  snprintf(freqString, sizeof(freqString), "%.2f", maxFrequency);
	  USART_Print(freqString);
}

int _write(int file, char *ptr, int len) // FOR PRINTF DEBUGGING
{
   for(int i=0; i<len; i++)
      ITM_SendChar((*ptr++));
   return len;
}

