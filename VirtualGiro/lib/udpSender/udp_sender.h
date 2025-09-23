#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include "lwip/udp.h"
#include "lwip/ip_addr.h"

// Inicializa o cliente UDP e retorna o ponteiro para o PCB
struct udp_pcb* udp_sender_init(void);

// Envia uma mensagem via UDP para o IP e porta especificados
void udp_sender_send(struct udp_pcb *pcb, const char *ip, uint16_t port, const char *msg);

#endif