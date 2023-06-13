#ifndef _USART_SEND_H
#define _USART_SEND_H

#include <stdio.h>
#include "main.h"
#include "app_error.h"

//#define my_debug

#define  mode_state     0x01
#define  manual_mode    0x01
#define  auto_mode      0x00

#define  power_state    0x10
#define  power_on       0x10
#define  power_off      0x00

#define  system_reset   0x20
#define  acc_run        0x40

extern uint8_t rx_data[20];
extern uint8_t rx_tmp_data;
extern uint8_t rx_sta;
extern uint8_t recvlen;
extern uint8_t rec_data_finish;

void uart_send_data_compose(void);
void usart_handle_receive_data(void);

void sendUartBytes(char * data, unsigned short datalen);

#endif