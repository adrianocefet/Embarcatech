#include "udp_sender.h"
#include <string.h>
#include <stdio.h>

struct udp_pcb* udp_sender_init(void) {
    struct udp_pcb *pcb = udp_new();
    if (!pcb) {
        printf("Erro ao criar PCB UDP\n");
        return NULL;
    }
    return pcb;
}

void udp_sender_send(struct udp_pcb *pcb, const char *ip, uint16_t port, const char *msg) {
    if (!pcb || !msg || !ip) return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg), PBUF_RAM);
    if (!p) {
        printf("Erro ao alocar pbuf\n");
        return;
    }

    memcpy(p->payload, msg, strlen(msg));

    ip_addr_t dest_ip;
    if (!ipaddr_aton(ip, &dest_ip)) {
        printf("IP inválido: %s\n", ip);
        pbuf_free(p);
        return;
    }

    err_t err = udp_sendto(pcb, p, &dest_ip, port);
    if (err != ERR_OK) {
        printf("Erro ao enviar UDP: %d\n", err);
    }

    pbuf_free(p);
}