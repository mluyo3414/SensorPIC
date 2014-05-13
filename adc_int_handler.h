/* 
 * File:   adc_int_handler.h
 * Author: Miguel S
 *
 * AD Interrupt handler
 * Created on February 5, 2014, 4:40 PM
 */

#ifndef ADC_INT_HANDLER_H
#define	ADC_INT_HANDLER_H

#include "maindefs.h"



int counter_interrupts = 0;
int display_counter = 0;
unsigned int ADCResult;
unsigned int frontSideADCResult;
unsigned int frontSensorADCResult;

unsigned char UART_Display [4];
unsigned char UART_Display_Front [4];

int bufferLen = 0;
int bufferLen1 = 0;
int bufferLen2 = 0;

unsigned int previousValue = 0;
unsigned int previousValue1 = 0;
// sensor buffers
unsigned char ADCBuffer0[I2C_MESSAGE_LENGTH];
unsigned char ADCBuffer1[I2C_MESSAGE_LENGTH];
unsigned int ADCBuffer0D[AVERAGE];
unsigned int ADCBuffer1D[I2C_MESSAGE_LENGTH];
unsigned int ADCBuffer0D2[AVERAGE];
unsigned char ADCBuffer01[I2C_MESSAGE_LENGTH];
unsigned char ADCBuffer11[I2C_MESSAGE_LENGTH];
unsigned int ADCBuffer0D1[AVERAGE];
unsigned int ADCBuffer1D1[I2C_MESSAGE_LENGTH];
unsigned char sensorBuffer[SENSOR_NUMBER + 1];
//sensor accumulators
int bottomSideCounter = 0;
int frontSideCounter = 0;
int frontSensorCounter = 0;
unsigned int bottomSideAccumulator = 0;
unsigned int frontSideAccumulator = 0;
unsigned int frontSensorAccumulator = 0;
unsigned char accumulatorHex = 0;
unsigned char bottomSideDistance = 0;
unsigned char frontSideDistance = 0;
unsigned char frontSensorDistance = 0;
//average
double bottomSideAverage = 0;
double frontSideAverage = 0;
double frontSensorAverage = 0;
//sensor display values for 7 segment LED
int display_side = 0;
int display_front = 0;

int dataDone = 0;
unsigned char sampleNumber = 0x0;
int prev_side_bottom = 0;
int prev_side_front = 0;
int prev_front = 0;


void adc_int_hand(int);
void adc_int_hand2(void);
void start_adc(void);
unsigned char get_value(int);

#endif	/* ADC_INT_HANDLER_H */

