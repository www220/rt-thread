#include <rtthread.h>
#include <lwip/sockets.h>

#include <finsh.h>
#include <shell.h>
#include "board.h"

#define TELNET_PORT         23
#define TELNET_MAX_CONNECTION	2
#define TELNET_USER			RTT_USER
#define TELNET_PASSWORD		RTT_PASS
#define TELNET_BUFFER_SIZE		1024

#define ISO_nl              0x0a
#define ISO_cr              0x0d

#define STATE_NORMAL        0
#define STATE_IAC           1
#define STATE_WILL          2
#define STATE_WONT          3
#define STATE_DO            4
#define STATE_DONT          5
#define STATE_CLOSE         6

#define TELNET_IAC          255
#define TELNET_WILL         251
#define TELNET_WONT         252
#define TELNET_DO           253
#define TELNET_DONT         254

#define NW_RX               0x01
#define NW_TX               0x02
#define NW_CLOSED           0x04
#define NW_MASK             (NW_RX | NW_TX | NW_CLOSED)

struct rb
{
    rt_uint16_t read_index, write_index;
    rt_uint8_t *buffer_ptr;
    rt_uint16_t buffer_size;
};

struct telnet_session
{
    struct rt_device device;

    struct rt_mutex mutex;
    struct rb rx_buf;
    struct rb tx_buf;
    int sockfd;

    /* telnet protocol */
    rt_uint8_t state;
	rt_uint8_t count;

};
struct telnet_session telnet;
static volatile int telnet_flag = 0;

/* 一个环形buffer的实现 */
/* 初始化环形buffer，size指的是buffer的大小。注：这里并没对数据地址对齐做处理 */
static void rb_init(struct rb* rb, rt_uint8_t *pool, rt_uint16_t size)
{
    RT_ASSERT(rb != RT_NULL);

    /* 对读写指针清零*/
    rb->read_index = rb->write_index = 0;

    /* 环形buffer的内存数据块 */
    rb->buffer_ptr = pool;
    rb->buffer_size = size;
}

/* 向环形buffer中写入一个字符 */
static rt_size_t rb_putchar(struct rb* rb, const rt_uint8_t ch)
{
    rt_uint16_t next;

    /* 判断是否有多余的空间 */
    next = rb->write_index + 1;
    if (next >= rb->buffer_size) next = 0;

    if (next == rb->read_index) return 0;

    /* 放入字符 */
    rb->buffer_ptr[rb->write_index] = ch;
    rb->write_index = next;

    return 1;
}

/* 从环形buffer中读出数据 */
static rt_size_t rb_get(struct rb* rb, rt_uint8_t *ptr, rt_uint16_t length)
{
    rt_size_t size;

    /* 判断是否有足够的数据 */
    if (rb->read_index > rb->write_index)
        size = rb->buffer_size - rb->read_index + rb->write_index;
    else
        size = rb->write_index - rb->read_index;

    /* 没有足够的数据 */
    if (size == 0) return 0;

    /* 数据不够指定的长度，取环形buffer中实际的长度 */
    if (size < length) length = size;

    if (rb->read_index > rb->write_index)
    {
        if (rb->buffer_size - rb->read_index > length)
        {
            /* read_index的数据足够多，直接复制 */
            memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
            rb->read_index += length;
        }
        else
        {
            /* read_index的数据不够，需要分段复制 */
            memcpy(ptr, &rb->buffer_ptr[rb->read_index],
                   rb->buffer_size - rb->read_index);
            memcpy(&ptr[rb->buffer_size - rb->read_index], &rb->buffer_ptr[0],
                   length - rb->buffer_size + rb->read_index);
            rb->read_index = length - rb->buffer_size + rb->read_index;
        }
    }
    else
    {
        /*
         * read_index要比write_index小，总的数据量够（前面已经有总数据量的判
         * 断），直接复制出数据。
         */
        memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        rb->read_index += length;
    }

    return length;
}

static rt_size_t rb_available(struct rb* rb)
{
    rt_size_t size;

    if (rb->read_index > rb->write_index)
        size = rb->buffer_size - rb->read_index + rb->write_index;
    else
        size = rb->write_index - rb->read_index;

    /* 返回ringbuffer中存在的数据大小 */
    return size;
}

/* RT-Thread Device Driver Interface */
static rt_err_t telnet_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t telnet_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t telnet_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t telnet_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_size_t result;

    /* read from rx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    result = rb_get(&telnet.rx_buf, buffer, size);
    rt_mutex_release(&telnet.mutex);

    return result;
}

static rt_size_t telnet_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_size_t i,result = 0;
    rt_uint8_t *buf = (rt_uint8_t *)buffer;

    /* write to tx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    for (i=0; i<size; i++)
    {
        if (rb_putchar(&telnet.tx_buf,buf[i]) == 0)
        {
            rt_mutex_release(&telnet.mutex);
            rt_thread_delay(10);
            rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
        }
        result++;
    }
    rt_mutex_release(&telnet.mutex);

    return result;
}

static rt_size_t telnet_write_inx (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    static rt_uint8_t changebuf[4096];
    rt_size_t i,index = 0,result = 0;

    for (i=0; i<size; i++)
    {
        if (((rt_uint8_t *)buffer)[i] == '\n') {
            changebuf[index++] = '\r';
        }
        if (index >= sizeof(changebuf)) {
            result += send(telnet.sockfd, changebuf, index, 0);
            index = 0;
        }
        changebuf[index++] = ((rt_uint8_t *)buffer)[i];
        if (index >= sizeof(changebuf)) {
            result += send(telnet.sockfd, changebuf, index, 0);
            index = 0;
        }
    }
    if (index) {
        result += send(telnet.sockfd, changebuf, index, 0);
    }

    return result;
}

static rt_err_t telnet_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    return RT_EOK;
}

/* send telnet option to remote */
static void telnet_send_option(rt_uint8_t option, rt_uint8_t value)
{
    rt_uint8_t optbuf[4];

    optbuf[0] = TELNET_IAC;
    optbuf[1] = option;
    optbuf[2] = value;
    optbuf[3] = 0;

    telnet_write_inx(&telnet.device, 0, optbuf, 3);
}

/* process rx data */
void telnet_process_rx(rt_uint8_t *data, rt_size_t length)
{
    rt_size_t rx_length, index;

    for (index = 0; index < length; index ++)
    {
        switch(telnet.state)
        {
        case STATE_IAC:
            if (*data == TELNET_IAC)
            {
                rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
                /* put buffer to ringbuffer */
                rb_putchar(&(telnet.rx_buf), *data);
                rt_mutex_release(&telnet.mutex);

                telnet.state = STATE_NORMAL;
            }
            else
            {
                /* set telnet state according to received package */
                switch (*data)
                {
                case TELNET_WILL: telnet.state = STATE_WILL; break;
                case TELNET_WONT: telnet.state = STATE_WONT; break;
                case TELNET_DO:   telnet.state = STATE_DO; break;
                case TELNET_DONT: telnet.state = STATE_DONT; break;
                default: telnet.state = STATE_NORMAL; break;
                }
            }
            break;
        
        /* don't option */
        case STATE_WILL:
        case STATE_WONT:
			if ((telnet.count & 0x01) == 0)
				telnet_send_option((telnet.state == STATE_WILL)?TELNET_DO:TELNET_DONT, *data);
			else
				telnet.count &= 0xfe;
            telnet.state = STATE_NORMAL;
            break;

        /* won't option */
        case STATE_DO:
        case STATE_DONT:
			if ((telnet.count & 0x02) == 0)
				telnet_send_option((telnet.state == STATE_DO)?TELNET_WILL:TELNET_WONT, *data);
			else
				telnet.count &= 0xfd;
            telnet.state = STATE_NORMAL;
            break;

        case STATE_NORMAL:
            if (*data == TELNET_IAC) telnet.state = STATE_IAC;
            else if (*data != '\r') /* ignore '\r' */
            {
                rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
                /* put buffer to ringbuffer */
                rb_putchar(&(telnet.rx_buf), ((*data)?(*data):('\n')));
                rt_mutex_release(&telnet.mutex);
            }
            break;
        }

        data ++;
    }

    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    /* get total size */
    rx_length = rb_available(&telnet.rx_buf);
    rt_mutex_release(&telnet.mutex);

    /* indicate there are reception data */
    if ((rx_length > 0) && (telnet.device.rx_indicate != RT_NULL))
        telnet.device.rx_indicate(&telnet.device, rx_length);

    return;
}

/* process tx data */
void telnet_process_tx()
{
    static rt_uint8_t recvbuf[4096];
    rt_size_t result;

    /* read from tx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    result = rb_get(&telnet.tx_buf, recvbuf, sizeof(recvbuf));
    rt_mutex_release(&telnet.mutex);

    if (result && telnet.sockfd >= 0)
        telnet_write_inx(&telnet.device, 0, recvbuf, result);
}

/* process netconn close */
void telnet_process_close()
{
    /* set console */
    rt_console_set_device(CONSOLE_DEVICE);
    /* set finsh device */
    finsh_set_device(FINSH_DEVICE_NAME);

    /* close connection */
    if (telnet.sockfd >= 0)
    {
        closesocket(telnet.sockfd);
        telnet.sockfd = -1;
        telnet.device.rx_indicate = NULL;
        telnet.device.tx_complete = NULL;

        rt_kprintf("resume console to uart.\n");
    }
}

/* telnet server thread entry */
static char clscode[] = {0x1B, 0x5B, 0x48, 0x1B, 0x5B, 0x4A, 0x00};
void telnet_thread(void* parameter)
{
    int numbytes;
    int sockfd, maxfdp1;
    struct sockaddr_in local;
    fd_set readfds, tmpfds;
    rt_uint32_t addr_len = sizeof(struct sockaddr);
    char * buffer = (char *) rt_malloc(TELNET_BUFFER_SIZE);
    struct timeval	tv;
    rt_tick_t tConnect = 0;
    int tPz = 0;
    char csUser[20] = {0};
    char csPass[20] = {0};
    rt_device_t cons;

    local.sin_port=htons(TELNET_PORT);
    local.sin_family=PF_INET;
    local.sin_addr.s_addr=INADDR_ANY;

    FD_ZERO(&readfds);
    FD_ZERO(&tmpfds);

    telnet_flag = 0;
    memset(&telnet, 0, sizeof(telnet));
    rb_init(&telnet.rx_buf, rt_malloc(TELNET_BUFFER_SIZE), TELNET_BUFFER_SIZE);
    rb_init(&telnet.tx_buf, rt_malloc(TELNET_BUFFER_SIZE*4), TELNET_BUFFER_SIZE*4);
    rt_mutex_init(&telnet.mutex, "telnet", RT_IPC_FLAG_FIFO);
    telnet.sockfd = -1;

    cons = rt_device_find(CONSOLE_DEVICE);
    if (cons)
        rt_device_open(cons, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM);

    tv.tv_sec = 1;
    tv.tv_usec = 0;
    sockfd=socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0)
    {
        rt_kprintf("create socket failed\n");
        return ;
    }

    bind(sockfd, (struct sockaddr *)&local, addr_len);
    listen(sockfd, TELNET_MAX_CONNECTION);

    FD_SET(sockfd, &readfds);

    /* register telnet device */
    telnet.device.type     = RT_Device_Class_Char;
    telnet.device.init     = telnet_init;
    telnet.device.open     = telnet_open;
    telnet.device.close    = telnet_close;
    telnet.device.read     = telnet_read;
    telnet.device.write    = telnet_write;
    telnet.device.control  = telnet_control;

    /* no private */
    telnet.device.user_data = &telnet;

    /* register telnet device */
    rt_device_register(&telnet.device, "telnet",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STREAM);

    for(;;)
    {
        if (telnet.sockfd >= 0 && tPz > 2 && cons)
        {
            numbytes=rt_device_read(cons, 0, buffer, TELNET_BUFFER_SIZE);
            if (numbytes > 5)
            {
                telnet_flag = 1;
                while (rt_device_read(cons, 0, buffer, TELNET_BUFFER_SIZE));
            }
        }
        if (telnet_flag && telnet.sockfd >= 0)
        {
            if (tPz > 2)
            {
                telnet_process_close();
            }
            else
            {
                closesocket(telnet.sockfd);
                telnet.sockfd = -1;
            }
        }
        telnet_flag = 0;

        /* get maximum fd */
        maxfdp1 = sockfd + 1;
        tmpfds=readfds;
        if (telnet.sockfd >= 0)
        {
            if (maxfdp1 < telnet.sockfd + 1)
                maxfdp1 = telnet.sockfd + 1;

            FD_SET(telnet.sockfd, &tmpfds);
            telnet_process_tx();
        }

        if (select(maxfdp1, &tmpfds, 0, 0, &tv) <= 0) continue;
        if(FD_ISSET(sockfd, &tmpfds))
        {
            int com_socket;
            struct sockaddr_in remote;

            com_socket = accept(sockfd, (struct sockaddr*)&remote, &addr_len);
            if(com_socket == -1)
            {
                rt_kprintf("Error on accept()\nContinuing...\n");
                continue;
            }
            else
            {
                if (telnet.sockfd < 0 || rt_tick_get()-tConnect > 60000)
                {
                    if (telnet.sockfd >= 0)
                    {
                        if (tPz > 2)
                        {
                            telnet_process_close();
                        }
                        else
                        {
                            closesocket(telnet.sockfd);
                            telnet.sockfd = -1;
                        }
                    }
                    rt_thread_delay(100);
                    while (telnet_read(&telnet.device, 0, buffer, TELNET_BUFFER_SIZE));
                    telnet_process_tx();telnet_process_tx();
                    telnet.sockfd = com_socket;

                    /* DO ECHO */
                    telnet_send_option(TELNET_DONT, 1);
                    telnet.count = 0x01;

                    /* WILL ECHO */
                    telnet_send_option(TELNET_WILL, 1);
                    telnet.count |= 0x02;

                    /* set init state */
                    telnet.state = STATE_NORMAL;
                    rt_thread_delay(100);

                    tPz = 0;
                    rt_memset(csUser,0,20);
                    rt_memset(csPass,0,20);

                    /* welcome */
                    telnet_write_inx(&telnet.device, 0, "-= welcome on RT-Thread TELNET server =-\n", 41);
                    telnet_write_inx(&telnet.device, 0, "login:", 6);
                }
                else
                {
                    send(com_socket, "wait for timeout\r\n", 18, 0);
                    rt_thread_delay(200);
                    closesocket(com_socket);
                }
            }
        }

        if (telnet.sockfd >=0 && FD_ISSET(telnet.sockfd, &tmpfds))
        {
            numbytes=recv(telnet.sockfd, buffer, TELNET_BUFFER_SIZE, 0);
            if(numbytes==0 || numbytes==-1)
            {
                if (tPz > 2)
                {
                    telnet_process_close();
                }
                else
                {
                    closesocket(telnet.sockfd);
                    telnet.sockfd = -1;
                }
                continue;
            }
            telnet_process_rx((rt_uint8_t *)buffer,numbytes);
            if (tPz != 2)
            {
                char chBuf[20];
                int i,readsize = telnet_read(&telnet.device,0,chBuf,20);
                int pos = (tPz==0)?(strlen(csUser)):(strlen(csPass));
                for (i=0; i<readsize; i++)
                {
                    if (chBuf[i] == '\r' || chBuf[i] == '\n')
                    {
                        tPz++;
                        if (tPz == 1)
                        {
                            telnet_write_inx(&telnet.device, 0, chBuf, readsize);
                            telnet_write_inx(&telnet.device, 0, "password:", 9);
                        }
                        else
                        {
                            telnet_write_inx(&telnet.device, 0,"\n",1);
                        }
                        break;
                    }
                    if (chBuf[i] < 48 && chBuf[i] > 122)
                        continue;
                    if (pos < 19)
                    {
                        if (tPz == 0)
                            csUser[pos] = chBuf[i];
                        else
                            csPass[pos] = chBuf[i];
                        pos++;
                    }
                }
                /* 密码不回显 */
                if (tPz == 0 && readsize)
                    telnet_write_inx(&telnet.device, 0, chBuf, readsize);
            }
            if (tPz == 2)
            {
                if (rt_strcmp(csPass, TELNET_PASSWORD)!=0 ||
                    rt_strcasecmp(csUser, TELNET_USER)!=0)
                {
                    telnet_write_inx(&telnet.device, 0, "Login incorrect\n", 16);
                    rt_thread_delay(200);
                    closesocket(telnet.sockfd);
                    telnet.sockfd = -1;
                    continue;
                }
                else
                {
                    tPz++;
                    telnet_write_inx(&telnet.device,0,clscode,strlen(clscode));
                    telnet_write_inx(&telnet.device,0,FINSH_PROMPT,strlen(FINSH_PROMPT));
                    rt_kprintf("new telnet connection, switch console to telnet.\n");

                    /* Process the new connection. */
                    /* set console */
                    rt_console_set_device("telnet");
                    /* set finsh device */
                    finsh_set_device("telnet");
                }
            }
            if (tPz >= 2)
            {
                tConnect = rt_tick_get();
            }
		}
    }
}

/* telnet server */
void telnetd()
{
    static rt_thread_t tid = RT_NULL;

    if (tid != RT_NULL) return;
    tid = rt_thread_create("telnetd", 
        telnet_thread, RT_NULL,
        4096, 30, 5);
    if (tid != RT_NULL) rt_thread_startup(tid);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void telnet_exit()
{
    telnet_flag = 1;
    while (telnet_flag)
        rt_thread_delay(100);
}

void cls()
{
    rt_kprintf(clscode);
}

FINSH_FUNCTION_EXPORT(telnetd, startup telnet server);
MSH_CMD_EXPORT(telnetd, start telnet server)
MSH_CMD_EXPORT(telnet_exit, exit telnet server)
MSH_CMD_EXPORT(cls, clear screen)
#endif
