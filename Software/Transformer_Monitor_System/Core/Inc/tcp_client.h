#ifndef __TCP_CLIENT_H
#define __TCP_CLIENT_H

#include "stm32f4xx_hal.h"
#include "lwip.h"
#include "tcp.h"
#include "string.h"
#include "main.h"

extern uint8_t server_ip[4];

static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
void tcp_client_init(void);


#endif
