#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "debug.h"
#include "i2cThread.h"
#include "my_i2c.h"

/**
 * USed to parse request from masterpic
 * @param fromMaster
 */

void parseRequest(unsigned char * fromMaster)
{

    masterRequest[0] = fromMaster[0];
    masterRequest[1] = fromMaster[1];
    masterRequest[2] = fromMaster[2];
    //handle error
    // check from 0x12 0x00 0x00
    if (masterRequest[0]!= 0x12 && masterRequest[1]!= 0x00 && masterRequest[2] != 0x00)
    {
        ToMainLow_sendmsg(3,I2C_READ_ERROR,(void*) masterRequest);
    }
  



}
