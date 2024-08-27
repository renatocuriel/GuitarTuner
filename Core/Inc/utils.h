/*
 * utils.h
 *
 *  Created on: May 30, 2024
 *      Author: thede
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"

#define NOTE_E2     83.0
#define NOTE_A   110.0
#define NOTE_D    147.0
#define NOTE_G   196.0
#define NOTE_B    247.0
#define NOTE_E3  330.0

typedef struct {
    float frequency;
    const char *note;
} FrequencyNotePair;

// Lookup table
extern FrequencyNotePair sixStringStandard[];

float getFrequencyFromNote(const char *note);

void USART_Print(const char* message);
void USART_ESC_Code(const char* message);
void computeFFT(void);
void printHeader(void);
void printFrequency(void);
//void processInput(command_t *cmd);
void updateLEDs(float currentFrequency, float desiredFrequency);
void USART_MoveCursor(int row, int col);
int _write(int file, char *ptr, int len);

#endif /* INC_UTILS_H_ */
