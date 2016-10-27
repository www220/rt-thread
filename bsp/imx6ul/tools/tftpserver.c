/*
 * IMPORTANT NOTE:
 *
 * The RAW version of the TFTP server does not deal with any timeouts.
 * Hence this software is unreliable in anything but a point to point
 * network connection with no packet loss.

 * This software is only intended as a simple demo of UDP programs under
 * RAW mode
 */

#include <stdio.h>
#include <string.h>

#include <rtthread.h>
#include <dfs_posix.h>
#include <finsh.h>

#include "lwip/inet.h"
#include "lwip/err.h"
#include "lwip/udp.h"

#include "tftpserver.h"

/* tftp_errorcode error strings */
const char *tftp_errorcode_string[] = 
{
    "not defined",
    "file not found",
    "access violation",
    "disk full",
    "illegal operation",
    "unknown transfer id",
    "file already exists",
    "no such user",
};

#define TFTP_ROOT       "/"

static unsigned tftp_port = 69;
static struct udp_pcb *tftpd_pcb = RT_NULL;

rt_inline void tftp_extract_filename(char *fname, char *buf)
{
    strcpy(fname, buf + 2);
}

rt_inline u16_t tftp_extract_block(char *buf)
{
    u16_t *b = (u16_t*)buf;
    return ntohs(b[1]);
}

rt_inline void tftp_set_errormsg(char *buf, const char *errormsg)
{
    strcpy(buf + 4, errormsg);
}

rt_inline void tftp_set_block(char *packet, int block)
{
    u16_t *p = (u16_t *)packet;
    p[1] = htons(block);
}

rt_inline void tftp_set_data_message(char *packet, char *buf, int n)
{
    memcpy(packet + 4, buf, n);
}

int tftp_is_correct_ack(char *buf, int block)
{
    /* first make sure this is a data ACK packet */
    if (buf[1] != TFTP_ACK)
        return 0;

    /* then compare block numbers */
    if (block != tftp_extract_block(buf))
        return 0;

    return 1;
}

err_t tftp_send_message(struct udp_pcb *pcb, ip_addr_t *to_ip, int to_port, char *buf, int buflen)
{
    err_t err;
    struct pbuf *p;

    /* form a pbuf */
    p = pbuf_alloc(PBUF_TRANSPORT, buflen, PBUF_POOL);
    if (!p) 
    {
        rt_kprintf("error allocating pbuf\r\n");
        return ERR_MEM;
    }
    memcpy(p->payload, buf, buflen);

    /* send message */
    err = udp_sendto(pcb, p, to_ip, to_port);

    /* free pbuf */
    pbuf_free(p);

    return err;
}

/* construct an error message into buf using err as the error code */
int tftp_construct_error_message(char *buf, tftp_errorcode err)
{
    int errorlen;

    buf[0] = 0; buf[1] = TFTP_ERROR;
    buf[2] = 0; buf[3] = (u8_t)err;

    tftp_set_errormsg(buf, tftp_errorcode_string[err]);
    errorlen = strlen(tftp_errorcode_string[err]);

    /* return message size */
    return 4 + errorlen + 1;
}

/* construct and send an error message back to client */
int tftp_send_error_message(struct udp_pcb *pcb, ip_addr_t *to, int to_port, tftp_errorcode err)
{
    char *buf;
    int n, result;

    buf = (char*) rt_malloc(512);
    if (!buf) return -1;

    n = tftp_construct_error_message(buf, err);
    result = tftp_send_message(pcb, to, to_port, buf, n);
    rt_free(buf);

    return result;
}

/* construct and send a data packet */
int tftp_send_data_packet(struct udp_pcb *pcb, ip_addr_t *to, int to_port, int block, char *buf, int buflen)
{
    char packet[TFTP_MAX_MSG_LEN];

    packet[0] = 0; packet[1] = TFTP_DATA;
    tftp_set_block(packet, block);
    tftp_set_data_message(packet, buf, buflen);

    return tftp_send_message(pcb, to, to_port, packet, buflen + 4);
}

int tftp_send_ack_packet(struct udp_pcb *pcb, ip_addr_t *to, int to_port, int block)
{
    char packet[TFTP_MAX_ACK_LEN];

    packet[0] = 0; packet[1] = TFTP_ACK;
    tftp_set_block(packet, block);

    return tftp_send_message(pcb, to, to_port, packet, TFTP_MAX_ACK_LEN);
}

void tftp_cleanup(struct udp_pcb *pcb, tftp_connection_args *args)
{
    /* cleanup the args */
    close(args->fd);
    rt_free(args);

    /* close the connection */
    udp_remove(pcb);
}

void tftp_send_next_block(struct udp_pcb *pcb, tftp_connection_args *args,
        ip_addr_t *to_ip, u16_t to_port)
{
    args->data_len = read(args->fd, args->data, TFTP_DATA_PACKET_MSG_LEN);
    if (args->data_len <= 0) 
    {
        rt_kprintf("closing connection, ret = %d\r\n", args->data_len);
        /* we are done */
        tftp_cleanup(pcb, args);
        return ;
    }

    /* send the data */
    tftp_send_data_packet(pcb, to_ip, to_port,
            args->block, args->data, args->data_len);
}

static void rrq_recv_callback(void *_args, struct udp_pcb *upcb,
                               struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    ip_addr_t dst_ip = *addr;
    tftp_connection_args *args = (tftp_connection_args *)_args;

    if (tftp_is_correct_ack(p->payload, args->block)) 
    {
        /* increment block # */
        ((tftp_connection_args *)args)->block++;
    } 
    else 
    {
        /* we did not receive the expected ACK, so
           do not update block #, thereby resending current block */
        rt_kprintf("incorrect ack received, resending last block\r\n");
    }

    pbuf_free(p);

    /* if the last read returned less than the requested number of bytes,
     * then we've sent the whole file and so we can quit
     */
    if (args->data_len < TFTP_DATA_PACKET_MSG_LEN)
    {
        tftp_cleanup(upcb, args);
        return;
    }
    
    tftp_send_next_block(upcb, args, &dst_ip, port);
}

int tftp_process_read(struct udp_pcb *pcb, ip_addr_t *to, int to_port, char *fname)
{
    int fd = -1;
    char *fn_ptr = NULL;
    tftp_connection_args *args = NULL;

    fn_ptr = rt_malloc(strlen(fname) + strlen(TFTP_ROOT) + 32);
    if (!fn_ptr) 
    {
        rt_kprintf("unable to allocate memory for filename\r\n");
        goto __exit;
    }

    sprintf(fn_ptr, "%s/%s", TFTP_ROOT, fname);
    fd = open(fn_ptr, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("unable to open file: %s\r\n", fname);
        goto __exit;
    }

    /* this function is called from a callback => interrupts are disabled
     * => we can use regular malloc
     */
    args = rt_malloc(sizeof *args);
    if (!args) 
    {
        rt_kprintf("unable to allocate memory for tftp args\r\n");
        goto __exit;
    }

    args->op = TFTP_RRQ;
    args->to_ip.addr = to->addr;
    args->to_port = to_port;
    args->fd = fd;

    /* set callback for receives on this pcb */
    udp_recv(pcb, rrq_recv_callback, args);

    /* initiate the transaction by sending the first block of data
     * further blocks will be sent when ACKs are received
     *  - the receive callbacks need to get the proper state
     */
    args->block = 1;
    tftp_send_next_block(pcb, args, to, to_port);

    if (fn_ptr) rt_free(fn_ptr);

    return 0;

__exit:
    if (fn_ptr) rt_free(fn_ptr);
    if (args) rt_free(args);
    if (fd >= 0) close(fd);

    tftp_send_error_message(pcb, to, to_port, TFTP_ERR_FILE_NOT_FOUND);
    udp_remove(pcb);

    return -1;
}

/* write callback */
void wrq_recv_callback(void *_args, struct udp_pcb *upcb,
                               struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    ip_addr_t dst_ip = *addr;
    tftp_connection_args *args = (tftp_connection_args *)_args;

    if (p->len != p->tot_len) 
    {
        rt_kprintf("ERROR: tftp server does not support chained pbuf's\r\n");
        pbuf_free(p);
        return;
    }

    /* make sure data block is what we expect */
    if ((p->len > 4) && (tftp_extract_block(p->payload) == (args->block + 1))) 
    {
        /* write the received data to the file */
        int n = write(args->fd, (u8_t*)p->payload+TFTP_DATA_PACKET_HDR_LEN,
                p->len-TFTP_DATA_PACKET_HDR_LEN);
        if (n != p->len-TFTP_DATA_PACKET_HDR_LEN) 
        {
            rt_kprintf("write to file error\r\n");
            tftp_send_error_message(upcb, &dst_ip, port, TFTP_ERR_DISKFULL);
            pbuf_free(p);
            tftp_cleanup(upcb, args);
            return ;
        }
        args->block++;
    }

    tftp_send_ack_packet(upcb, &dst_ip, port, args->block);

    /* if the last read returned less than the requested number of bytes,
     * then we've sent the whole file and so we can quit
     */
    if (p->len < TFTP_DATA_PACKET_MSG_LEN)
    {
        tftp_cleanup(upcb, args);
        return ;
    }

    pbuf_free(p);
}

/* write data coming via sd to file *fname */
int tftp_process_write(struct udp_pcb *pcb, ip_addr_t *to, int to_port, char *fname)
{
    int fd = -1;
    char *fn_ptr = NULL;
    tftp_connection_args *args = NULL;
    tftp_errorcode error_code;

    fn_ptr = (char *)rt_malloc(strlen(fname) + strlen(TFTP_ROOT) + 32);
    if (!fn_ptr)
    {
        printf("unable allocate filename\n");
        error_code = TFTP_ERR_NOTDEFINED;
        goto __exit;
    }

    sprintf(fn_ptr, "%s/%s", TFTP_ROOT, fname);
    fd = open(fn_ptr, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) 
    {
        rt_kprintf("unable to open file %s for writing\r\n", fn_ptr);
        error_code = TFTP_ERR_DISKFULL;
        goto __exit;
    }

    /* this function is called from a callback => interrupts are disabled
     * => we can use regular malloc
     */
    args = rt_malloc(sizeof *args);
    if (!args) 
    {
        rt_kprintf("unable to allocate memory for tftp args\r\n");
        error_code = TFTP_ERR_FILE_NOT_FOUND;
        goto __exit;
    }

    args->op = TFTP_WRQ;
    args->to_ip.addr = to->addr;
    args->to_port = to_port;
    args->fd = fd;
    args->block = 0;

    /* set callback for receives on this pcb */
    udp_recv(pcb, wrq_recv_callback, args);

    /* initiate the transaction by sending the first ack */
    tftp_send_ack_packet(pcb, to, to_port, args->block);

    if (fn_ptr) rt_free(fn_ptr);

    return 0;

__exit:
    if (fd >= 0) close(fd);
    if (fn_ptr) rt_free(fn_ptr);
    if (args)   rt_free(args);

    tftp_send_error_message(pcb, to, to_port, error_code);
    udp_remove(pcb);
    return -1;
}

/* for each new request (data in p->payload) from addr:port,
 * create a new port to serve the response, and start the response
 * process
 */
static void process_tftp_request(struct pbuf *p, ip_addr_t *addr, u16_t port)
{
    tftp_opcode op = (tftp_opcode)((u8_t*)p->payload)[1];
    char *fn_ptr = NULL;
    struct udp_pcb *pcb = NULL;
    err_t err;

    fn_ptr = (char*)rt_malloc(256);
    if (!fn_ptr) return ;

    /* create new UDP PCB structure */
    pcb = udp_new();
    if (!pcb) 
    {
        rt_kprintf("Error creating PCB. Out of Memory\r\n");
        goto __exit;
    }

    /* bind to port 0 to receive next available free port */
    err = udp_bind(pcb, IP_ADDR_ANY, 0);
    if (err != ERR_OK) 
    {
        rt_kprintf("Unable to bind to port %d: err = %d\r\n", port, err);
        udp_remove(pcb);
        goto __exit;
    }

    switch (op) 
    {
    case TFTP_RRQ:
        tftp_extract_filename(fn_ptr, p->payload);
        printf("TFTP RRQ (read request): %s\r\n", fn_ptr);
        tftp_process_read(pcb, addr, port, fn_ptr);
        break;

    case TFTP_WRQ:
        tftp_extract_filename(fn_ptr, p->payload);
        printf("TFTP WRQ (write request): %s\r\n", fn_ptr);
        tftp_process_write(pcb, addr, port, fn_ptr);
        break;

    default:
        /* send a generic access violation message */
        tftp_send_error_message(pcb, addr, port, TFTP_ERR_ACCESS_VIOLATION);
        printf("TFTP unknown request op: %d\r\n", op);
        udp_remove(pcb);
        break;
    }

__exit:
    if (fn_ptr) rt_free(fn_ptr);
}

/* the tftp_recv_callback function is called when there is a packet received
 * on the main tftp server port (69)
 */
static void tftp_recv_callback(void *arg, struct udp_pcb *upcb,
                               struct pbuf *p, const ip_addr_t *_addr, u16_t port)
{
	ip_addr_t addr = *_addr;

    /* process new connection request */
    process_tftp_request(p, &addr, port);

    pbuf_free(p);
}

int start_tftp_application(void)
{
    err_t err;
    struct udp_pcb *pcb;
    unsigned int port = tftp_port;

    /* create new UDP PCB structure */
    pcb = udp_new();
    if (!pcb) 
    {
        rt_kprintf("Error creating PCB. Out of Memory\r\n");
        return -1;
    }

    /* bind to @port */
    err = udp_bind(pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) 
    {
        rt_kprintf("Unable to bind to port %d: err = %d\r\n", port, err);
        return -2;
    }

    udp_recv(pcb, tftp_recv_callback, NULL);
    tftpd_pcb = pcb;
    rt_kprintf("tftp server start @ root - %s\n", TFTP_ROOT);

    return 0;
}

void print_tftp_app_header()
{
    rt_kprintf("%20s %6d %s\r\n", "tftp server",
        tftp_port,
        "$ tftp -i 192.168.1.10 PUT <source-file>");
}

int tftpd(int argc, char** argv)
{
    if (argc == 2)
    {
        if (strcmp(argv[1], "start") == 0)
        {
            /* start tftp daemon */
            if (!tftpd_pcb) start_tftp_application();

            return 0;
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            /* stop ftp daemon */
            if (tftpd_pcb) 
            {
                udp_remove(tftpd_pcb);
                tftpd_pcb = NULL;
            }

            return 0;
        }
    }

    rt_kprintf("tftpd start|stop\n");

    return 0;
}
MSH_CMD_EXPORT(tftpd, tftp server);
