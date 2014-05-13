#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "debug.h"

static uart_comm *uc_ptr;
static unsigned char buf[10];
static unsigned char count = 0;


void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->Tx_buflen = 0;
    uc_ptr->Rx_buflen = 0;
    uc_ptr->msg_length = 0;
}

void uart_retrieve_buffer(int length, unsigned char* msgbuffer) {

    uc_ptr->Tx_buflen = 0;
    uc_ptr->msg_length = length;

    int i = 0;
    for (; i < length + 1; i++) {
        uc_ptr->Tx_buffer[i] = msgbuffer[i];
    }
}
void uart_send_int_handler() {

    if (uc_ptr->Tx_buflen == uc_ptr->msg_length) {
        PIE1bits.TX1IE = 0; // Clear TXIE to end write.
        uc_ptr->Tx_buflen = 0;
    } else {
        WriteUSART(uc_ptr->Tx_buffer[uc_ptr->Tx_buflen]);
        uc_ptr->Tx_buflen++;
    }
}


void uart_write(unsigned char length, unsigned char *msg) {

    unsigned char i = 0;
    for (i = 0; i < length; i++) {
        #ifdef __USE18F26J50
        while(Busy1USART());
        Write1USART(msg[i]);
        #else
        #ifdef __USE18F46J50
         while(BusyUSART());
            WriteUSART(msg[i]);
        #else
            while(BusyUSART());
            WriteUSART(msg[i]);
        #endif
        #endif

}
}
void uart_recv_int_handler() {
#ifdef __USE18F26J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
#ifdef __USE18F46J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
    if (DataRdyUSART()) {
        uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = ReadUSART();
#endif
#endif

        uc_ptr->Rx_buflen++;
        // check if a message should be sent
        if (uc_ptr->Rx_buflen == MAXUARTBUF) {
            ToMainLow_sendmsg(uc_ptr->Rx_buflen, MSGT_UART_DATA, (void *) uc_ptr->Rx_buflen);
            uc_ptr->Rx_buflen = 0;
        }
    }
#ifdef __USE18F26J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
#ifdef __USE18F46J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
    if (USART_Status.OVERRUN_ERROR == 1) {
#endif
#endif
        // we've overrun the USART and must reset
        // send an error message for this
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
}


