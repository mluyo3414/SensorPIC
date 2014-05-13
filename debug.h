/*
 * File:   debug.h
 * Author: ECE4534
 * Instrumentation code
 * Created on February 10, 2014, 8:08 PM
 */

#ifndef DEBUG_H
#define DEBUG_H



// Define this to turn on instrumentation
#define DO_DEBUG
//      Use definitions to make debugging configurable


#define CLOSE_WALL LATBbits.LATB3
#define FAR_WALL LATBbits.LATB4
#define CLOSE_S1 LATBbits.LATB1
#define FAR_S1 LATBbits.LATB2



#define ADC_INT LATCbits.LATC0 //4
#define TIMER0_ISR LATCbits.LATC1 //5
#define ADC_START LATCbits.LATC2 // 6
#define SENSOR_DATA LATCbits.LATC5 //7

#define UART  LATDbits.LATD2 // 8




//      Define our debugging functions
#ifdef  DO_DEBUG

#define DEBUG_ON(a){ \
    a = 1; \
    }

#define DEBUG_OFF(a){ \
    a = 0; \
    }

#else
}
#define DEBUG_ON(a)
#define DEBUG_OFF(a)
#endif
#endif