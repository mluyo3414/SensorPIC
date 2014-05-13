/*
 * File:   adc_int_handler.h
 * Author: Miguel S
 * I2C communication thread
 * Created on February 5, 2014, 4:40 PM
 */
#include "my_i2c.h"
#ifndef I2CTHREAD_H
#define	I2CTHREAD_H


unsigned char masterRequest[3];
void parseRequest(unsigned char*);












#endif	/* I2CTHREAD_H */