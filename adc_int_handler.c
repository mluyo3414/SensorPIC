#include "maindefs.h"
#ifndef __XC8
#include <adc.h>
#else
#include <plib/adc.h>
#endif
#include "adc_int_handler.h"
#include "messages.h"
#include "debug.h"
#include "my_i2c.h"
#include "my_uart.h"
#include	<stdio.h>
#include	<stdarg.h>

/*
 AD interrupt handler

 */
void adc_int_hand(int sensor) {
    DEBUG_ON(ADC_INT);
    DEBUG_OFF(ADC_INT);


  // SIDE SENSOR (bottom)  
 if(sensor==1)
    {


    ADCResult = 0;
    ADCResult = (int) ReadADC() >> 2;


        //holds values as ints
        ADCBuffer0D[bottomSideCounter] = ADCResult;
        //adding for average
        bottomSideAccumulator = bottomSideAccumulator + ADCResult;
        //increasing counter
        bottomSideCounter++;
        //if buffer is full
        if (bottomSideCounter > AVERAGE) {
            //reset counter and get average
            bottomSideCounter = 0;
             //increase counter to send to main
	     dataDone++;

            //average as a double
            bottomSideAverage = bottomSideAccumulator / AVERAGE;
            //look up table

             if (bottomSideAverage <= 205 && bottomSideAverage >= 188) {
                bottomSideDistance = 0x10;
            } else if (bottomSideAverage < 188 && bottomSideAverage >= 160) {
                bottomSideDistance = 0x12;
            } else if (bottomSideAverage < 160 && bottomSideAverage >= 142) {
                bottomSideDistance = 0x14;

            } else if (bottomSideAverage < 142 && bottomSideAverage >= 127) {
                bottomSideDistance = 0x16;

            } else if (bottomSideAverage < 127 && bottomSideAverage >= 117) {
                bottomSideDistance = 0x18;

            } else if (bottomSideAverage < 117 && bottomSideAverage >= 108) {
                bottomSideDistance = 0x20;

            } else if (bottomSideAverage < 108 && bottomSideAverage >= 98) {
                bottomSideDistance = 0x22;

            } else if (bottomSideAverage < 98 && bottomSideAverage >= 92) {
                bottomSideDistance = 0x24;

            } else if (bottomSideAverage < 92 && bottomSideAverage >= 85) {
                bottomSideDistance = 0x26;

            }
              else if (bottomSideAverage < 85 && bottomSideAverage >= 80) {
                bottomSideDistance = 0x28;
              }
             else if (bottomSideAverage < 80 && bottomSideAverage >= 75) {
                bottomSideDistance = 0x30;
              }
              else if (bottomSideAverage < 75 && bottomSideAverage >= 70) {
                bottomSideDistance = 0x32;
              }
              else if (bottomSideAverage < 70 && bottomSideAverage >= 63) {
                bottomSideDistance = 0x36;
              }
              else if (bottomSideAverage < 63 && bottomSideAverage >= 58) {
                bottomSideDistance = 0x40;
              }
             else if (bottomSideAverage < 58 && bottomSideAverage >= 53) {
                bottomSideDistance = 0x44;
              }


            else {
                bottomSideDistance = 0xFF;
            }


            ////////DISPLAY in cm

            // first digit in Hex
           UART_Display [0] = (bottomSideDistance&0b11110000)>>4;
           UART_Display [1] = (bottomSideDistance&0b00001111);

            

            //light red ( closer than 16 cm or greater than 44)
            if ((bottomSideDistance >= 0x16) && (bottomSideDistance<40)) {
                DEBUG_ON(CLOSE_WALL);
                DEBUG_OFF(FAR_WALL);

            }//light is yellow if we do not need adjustments
            else {
                DEBUG_OFF(CLOSE_WALL);
                DEBUG_ON(FAR_WALL);
            }
            //reset everything
            bottomSideAccumulator = 0;
            bottomSideAverage = 0;

        }

}

    //Side Sensor B Front
 else if (sensor ==2)
 {


     frontSideADCResult = 0;
     frontSideADCResult = (int) ReadADC() >> 2;




        //holds values as ints
        ADCBuffer0D1[frontSideCounter] = frontSideADCResult;
        //adding for average
        frontSideAccumulator = frontSideAccumulator + frontSideADCResult;

        //increasing counter
        frontSideCounter++;
        if (frontSideCounter > AVERAGE) {
            //reset counter and get average
            frontSideCounter = 0;
            frontSideAverage = frontSideAccumulator / AVERAGE;
             //increase counter to send to main
            dataDone++;

            //look up table
             if (frontSideAverage <= 205 && frontSideAverage >= 188) {
                frontSideDistance = 0x10;
            } else if (frontSideAverage < 188 && frontSideAverage >= 160) {
                frontSideDistance = 0x12;
            } else if (frontSideAverage < 160 && frontSideAverage >= 142) {
                frontSideDistance = 0x14;

            } else if (frontSideAverage < 142 && frontSideAverage >= 127) {
                frontSideDistance = 0x16;

            } else if (frontSideAverage < 127 && frontSideAverage >= 114) {
                frontSideDistance = 0x18;

            } else if (frontSideAverage < 114 && frontSideAverage >= 103) {
                frontSideDistance = 0x20;

            } else if (frontSideAverage < 103 && frontSideAverage >= 95) {
                frontSideDistance = 0x22;

            } else if (frontSideAverage < 95 && frontSideAverage >= 90) {
                frontSideDistance = 0x24;

            } else if (frontSideAverage < 90 && frontSideAverage >= 84) {
                frontSideDistance = 0x26;

            }
              else if (frontSideAverage < 84 && frontSideAverage >= 79) {
                frontSideDistance = 0x28;
              }
             else if (frontSideAverage < 79 && frontSideAverage >= 75) {
                frontSideDistance = 0x30;
              }
              else if (frontSideAverage < 75 && frontSideAverage >= 70) {
                frontSideDistance = 0x32;
              
              }
              else if (frontSideAverage < 69 && frontSideAverage >= 64) {
                frontSideDistance = 0x36;
              }
              else if (frontSideAverage < 64 && frontSideAverage >= 59) {
                frontSideDistance = 0x40;
              }
              else if (frontSideAverage < 59 && frontSideAverage >= 53) {
                frontSideDistance = 0x44;
              }
              
               else
               {
                     frontSideDistance = 0xFF;
               }

             if (display_side ==1)
             {
            // second digit
            UART_Display [2] = (frontSideDistance&0b11110000)>>4;
          //third digit
           UART_Display [3] = frontSideDistance&0b00001111;


           ToMainLow_sendmsg(4, MSGT_UART_WRITE_SIDE, (void *) UART_Display);
             }


            //light red out of range
            if ((frontSideDistance >= 0x16)&& (frontSideDistance<40)) {
                DEBUG_ON(CLOSE_S1);
                DEBUG_OFF(FAR_S1);

            }//light is yellow in range
            else {
                DEBUG_OFF(CLOSE_S1);
                DEBUG_ON(FAR_S1);

            }
            frontSideAccumulator = 0;
            frontSideAverage = 0;
        }

    }

    // Front sensor
    else if (sensor == 0) {

   
        frontSensorADCResult = 0;

        frontSensorADCResult = (int) ReadADC() >> 2;


        //holds values as ints
        ADCBuffer0D2[frontSensorCounter] = frontSensorADCResult;
        //adding for average
        frontSensorAccumulator = frontSensorAccumulator + frontSensorADCResult;
        //increasing counter
        frontSensorCounter++;
        if (frontSensorCounter > AVERAGE) {
            //reset counter and get average
            frontSensorCounter = 0;
            frontSensorAverage = frontSensorAccumulator / AVERAGE;
            //increase counter to send to main
               dataDone++;
            //look up table
              if (frontSensorAverage<=247 && frontSensorAverage >=233 )
              {
            frontSensorDistance= 0x12;
             }
            else if (frontSensorAverage<233 && frontSensorAverage >=226 )
              {
            frontSensorDistance= 0x14;
             }
            else if (frontSensorAverage<226 && frontSensorAverage >=219 )
              {
            frontSensorDistance= 0x16;
             }
            else if (frontSensorAverage<219 && frontSensorAverage >=210 )
              {
            frontSensorDistance= 0x18;
             }
             else if (frontSensorAverage<210 && frontSensorAverage >=198 )
              {
            frontSensorDistance= 0x20;
             }
             else if (frontSensorAverage<198 && frontSensorAverage >=190 )
              {
            frontSensorDistance= 0x22;
             }
             else if (frontSensorAverage<190 && frontSensorAverage >=180 )
              {
            frontSensorDistance= 0x24;
             }
            else if (frontSensorAverage<180 && frontSensorAverage >=173)
              {
            frontSensorDistance= 0x26;
             }
            else if (frontSensorAverage<173 && frontSensorAverage >=162 )
              {
            frontSensorDistance= 0x28;
             }
             else if (frontSensorAverage<162 && frontSensorAverage >=153 )
              {
            frontSensorDistance= 0x30;
             }
             else if (frontSensorAverage<153 && frontSensorAverage >=144 )
              {
            frontSensorDistance= 0x32;
             }
             else if (frontSensorAverage<144 && frontSensorAverage >=137 )
              {
            frontSensorDistance= 0x34;
             }
             else if (frontSensorAverage<137 && frontSensorAverage >=129 )
              {
            frontSensorDistance= 0x36;
             }
             else if (frontSensorAverage<129 && frontSensorAverage >=123 )
              {
            frontSensorDistance= 0x38;
             }
             else if (frontSensorAverage<123 && frontSensorAverage >=115 )
              {
            frontSensorDistance= 0x40;
             }
            else if (frontSensorAverage<115 && frontSensorAverage >=111)
              {
            frontSensorDistance= 0x42;
             }
            else if (frontSensorAverage<111 && frontSensorAverage >=104 )
              {
            frontSensorDistance= 0x44;
             }
            else if (frontSensorAverage<104 && frontSensorAverage >=97 )
              {
            frontSensorDistance= 0x46;
             }
            else if (frontSensorAverage<97 && frontSensorAverage >=94 )
              {
            frontSensorDistance= 0x48;
             }
            else if (frontSensorAverage<94 && frontSensorAverage >=89 )
              {
            frontSensorDistance= 0x50;
             }
             
            else
            {
                frontSensorDistance =0xFF;
            }


            if (display_front == 1) {
                unsigned char clearDisplay [1];
                clearDisplay[0] = 0x76;
  

                // (F)
                UART_Display_Front [0] = 0xF;
                //(S)
                UART_Display_Front [1] = 0x5;
                // first digit in Cm
                UART_Display_Front [2] = (frontSensorDistance & 0b11110000) >> 4;
                //second digit in Cm
                UART_Display_Front [3] = (frontSensorDistance & 0b00001111);
              //  uart_write(4, &UART_Display_Front);
              
            ToMainLow_sendmsg(4, MSGT_UART_WRITE_FRONT, (void *) UART_Display_Front);
 
               

            }

            frontSensorAccumulator = 0;
            frontSensorAverage = 0;

        }


    }



}
/**
 *
 * Start sampling triggered by the timer
 */

    void start_adc(void) {
    DEBUG_ON(ADC_START);
    DEBUG_OFF(ADC_START);

    //side sensor bottom channel
    if (counter_interrupts == 0) {
        SetChanADC(ADC_CH0);
        counter_interrupts++;
    } else if (counter_interrupts == 1) {
        //side  sensor  top channel
        SetChanADC(ADC_CH2);
        counter_interrupts++;
    }// front sensor
    else if (counter_interrupts == 2) {
        SetChanADC(ADC_CH4);
        counter_interrupts++;
    }


    if (counter_interrupts > 2) {

        counter_interrupts = 0;
    }
       //Data is ready send it to main
    if (dataDone==SENSOR_NUMBER)
    {
        sampleNumber++;
        dataDone =0;
        sensorBuffer[0] = bottomSideDistance;
        sensorBuffer[1] = frontSideDistance;
        sensorBuffer[2] = frontSensorDistance;
        sensorBuffer[3] = sampleNumber;

 
     ToMainLow_sendmsg(4, MSGT_SENSORS, (void *) sensorBuffer);

    // clear sensor values
    bottomSideDistance = 0;
    frontSideDistance = 0;
    frontSensorDistance = 0;


    }


    display_counter++;
    //display side 3 secs
    if (display_counter < 300) {
        display_side = 1;
        display_front = 0;
    }//display front 3 secs
        else {

            display_front = 1;
            display_side = 0;
            if (display_counter == 600) {
                display_counter = 0;
            }
        }
        //start sampling
        ADCON0bits.GO = 1;
    }


