#ifndef __TCP_CLIENT_H
#define __TCP_CLIENT_H

#include "stm32f4xx_hal.h"
#include "lwip.h"
#include "udp.h"
#include "string.h"
#include "main.h"

extern uint8_t server_ip[4];
extern uint16_t server_port;
extern struct udp_pcb *upcb;

static void udp_receive_callback(void *arg, struct udp_pcb *upcb,
            struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_client_send(char *pData);
void udp_client_init(void);
#endif
