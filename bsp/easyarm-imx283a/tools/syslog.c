#include <rtthread.h>
#include <rthw.h>
#include <rtm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <dfs_posix.h>
#include "board.h"

typedef int (*PRINT_INFO)(const char*, int);
int g_debug_level = rtt_LogNotice;
PRINT_INFO g_print_info = 0;

#define ONCE_BUF_SIZE 32768
#define ONCE_FILE_SIZE 131072
#define FILE_MAX_COUNT 10

ALIGN(RT_ALIGN_SIZE)
static char bg_syslog_func_stack[4096];
struct rt_thread thread_syslog_func;
static struct rt_mutex syslog_mutex;
struct rt_semaphore syslog_sem;
volatile int g_nSaveSysLog = 0;

struct sysloginfo
{
    struct sysloginfo *next;
    char buf[ONCE_BUF_SIZE];
    int len;
    rt_tick_t time;
};
static struct sysloginfo *syslog_base = NULL;
static struct sysloginfo* getlastsyslog(int *count)
{
    struct sysloginfo *lst = syslog_base;
    *count = 1;
    while (lst->next)
    {
        lst = lst->next;
        *count = *count + 1;
    }
    return lst;
}

static int rolloverFiles(const char* filename, unsigned int maxBackupIndex)
{
    int i,ret = 0;
    char source_oss[100];
    char target_oss[100];

    for (i = maxBackupIndex - 1; i >= 0; --i)
    {
        if (i == 0)
        {
            sprintf(source_oss, "%s", filename);
            sprintf(target_oss, "%s.%d", filename, i+1);
        }
        else
        {
            sprintf(source_oss, "%s.%d", filename, i);
            sprintf(target_oss, "%s.%d", filename, i+1);
        }

        ret = unlink(target_oss);
        ret = rename(source_oss, target_oss);
    }
    return ret;
}
static int SaveFile(const char* info, int len)
{
    //rewrite 3tims
    int i,nSize;
    char sz[512];

    sprintf(sz, "%s/loginfo",rttLogFileDir);
    for (i=0; i<3; i++){
        FILE *file = fopen(sz, "a+");
        if (file) {
            fwrite(info, len, 1, file);
            nSize = ftell(file);
            fclose(file);
            if (nSize >= ONCE_FILE_SIZE)
                rolloverFiles(sz, FILE_MAX_COUNT);
            break;
        } else {
            rt_thread_delay(1000);
        }
    }
    return 0;
}

static void bg_syslog_func(void *parameter)
{
    struct sysloginfo *info;
    info = (struct sysloginfo *)malloc(sizeof(struct sysloginfo));
    if (info == NULL)
        return;
    while(1)
    {
        info->len = 0;
        rt_mutex_take(&syslog_mutex, RT_WAITING_FOREVER);
        //save over 2 file
        if (syslog_base && syslog_base->next)
        {
            struct sysloginfo *lst = syslog_base;
            memcpy(info->buf,lst->buf,lst->len+1);
            info->len = lst->len;
            syslog_base = lst->next;
            free(lst);
        }
        else
        {
            //save over 300s
            unsigned int now_time = rt_tick_get();
            if (syslog_base->time != 0 && (g_nSaveSysLog || now_time-syslog_base->time >= 300000))
            {
                memcpy(info->buf,syslog_base->buf,syslog_base->len+1);
                info->len = syslog_base->len;
                syslog_base->len = 0;
                syslog_base->time = 0;
            }
        }
        rt_mutex_release(&syslog_mutex);
        //±£´æÎÄ¼þ
        if (info->len)
        {
            SaveFile(info->buf, info->len);
            info->len = 0;
        }
        else
        {
            g_nSaveSysLog = 0;
        }
        rt_sem_take(&syslog_sem, 1000);
    }
}

char *getunsave_loginfo()
{
    char *info = RT_NULL;
    rt_mutex_take(&syslog_mutex, RT_WAITING_FOREVER);
    if (syslog_base && syslog_base->len > 0)
    {
        info = (char *)malloc(syslog_base->len+1);
        if (info)
            memcpy(info,syslog_base->buf,syslog_base->len+1);
    }
    rt_mutex_release(&syslog_mutex);
    return info;
}
RTM_EXPORT(getunsave_loginfo);

static int debug_printf_output(const char *info, int len)
{
    struct sysloginfo *syslog;
    int count = 0;
    rt_mutex_take(&syslog_mutex, RT_WAITING_FOREVER);

    syslog = getlastsyslog(&count);
    if (syslog == NULL)
    {
        rt_mutex_release(&syslog_mutex);
        return 0;
    }
    if (syslog->len+len >= ONCE_BUF_SIZE)
    {
        struct sysloginfo *syslog_malloc;
        //remove oldest syslog
        if (count > 10)
        {
            struct sysloginfo *lst = syslog_base;
            if (lst == NULL)
            {
                rt_mutex_release(&syslog_mutex);
                return 0;
            }
            syslog_base = lst->next;
            free(lst);
        }
        //alloc new syslog
        syslog_malloc = (struct sysloginfo *)malloc(sizeof(struct sysloginfo));
        if (syslog_malloc == NULL)
        {
            rt_mutex_release(&syslog_mutex);
            return 0;
        }
        syslog->next = syslog_malloc;
        syslog = syslog_malloc;
        syslog->next = 0;
        syslog->time = 0;
        syslog->len = 0;
    }
    memcpy(syslog->buf+syslog->len,info,len+1);
    syslog->len += len;
    if (syslog->time == 0)
        syslog->time = rt_tick_get();
    rt_mutex_release(&syslog_mutex);
    return len;
}

int debug_printf_init()
{
    rt_mutex_init(&syslog_mutex, "syslog", RT_IPC_FLAG_FIFO);
    rt_sem_init(&syslog_sem, "syslog", 0, RT_IPC_FLAG_FIFO);

    rt_thread_init(&thread_syslog_func,
        "syslog",
        bg_syslog_func,
        RT_NULL,
        &bg_syslog_func_stack[0],
        sizeof(bg_syslog_func_stack),RT_THREAD_PRIORITY_MAX - 2,20);
    rt_thread_startup(&thread_syslog_func);

    syslog_base = (struct sysloginfo *)malloc(sizeof(struct sysloginfo));
    if (syslog_base == NULL)
        return -1;
    syslog_base->next = 0;
    syslog_base->time = 0;
    syslog_base->len = 0;
    g_print_info = debug_printf_output;
    return 0;
}

extern int __rt_ffs(int value);
int debug_printf_ap(int debug_level, const char *module, const char *format, va_list ap)
{
    if(debug_level <= g_debug_level)
    {
        int nbuf,info;
        struct timeval tp;
        struct tm now;
		char *bufdata;
		int buflen;

#define _DEF_MSGBUF 4096
        char *msg_info = (char *)rt_malloc(_DEF_MSGBUF);
        if (!msg_info)
            return 0;

        gettimeofday(&tp,NULL);
        localtime_r(&tp.tv_sec, &now);

        nbuf = sprintf(msg_info, "%04d-%02d-%02d %02d:%02d:%02d.%03d %-16s %02d ", now.tm_year+1900, now.tm_mon+1, now.tm_mday,
            now.tm_hour, now.tm_min, now.tm_sec, (int)(tp.tv_usec/1000l), module, debug_level);

        vsnprintf(msg_info+nbuf, sizeof(_DEF_MSGBUF)-nbuf, format, ap);
        msg_info[_DEF_MSGBUF-1] = 0;

		buflen = 0;
		bufdata = msg_info;
		while(*bufdata){
			if (*bufdata == '\r' || *bufdata == '\n')
				*bufdata = ' ';
			buflen++;
			bufdata++;
		}
		if (buflen)
			msg_info[buflen-1] = '\n';

        if (g_print_info)
            nbuf = g_print_info(msg_info, buflen);

        rt_free(msg_info);
        return nbuf;
    }

    return 0;
}
RTM_EXPORT(debug_printf_ap)

int debug_printf_def(int debug_level, const char *module, const char *format, ...)
{
    int ret;
    va_list ap;
    va_start(ap, format);
    ret = debug_printf_ap(debug_level, module, format, ap);
    va_end(ap);
    return ret;
}
RTM_EXPORT(debug_printf_def)

#if defined(FINSH_USING_MSH)
#include <shell.h>
#include <finsh.h>

int cmd_sysloglevel(int argc, char** argv)
{
    int level = -1;
    if (argc > 1)
        level = atol(argv[1]);
    if (level >= 0)
        g_debug_level = level;
    rt_kprintf("syslog level:%d\n",g_debug_level);
    return 0;
}

int cmd_syslogsave(int argc, char** argv)
{
    int i;
    g_nSaveSysLog = 1;
    rt_sem_release(&syslog_sem);
    g_nSaveSysLog = 1;
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    for (i=0; i<100 && g_nSaveSysLog; i++)
        rt_thread_delay(100);
    if (g_nSaveSysLog)
        rt_kprintf("syslog failed to save!\n");
    else
        rt_kprintf("syslog saved!\n");
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_sysloglevel, __cmd_sysloglevel, Syslog Level.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_syslogsave, __cmd_syslogsave, Syslog Save.)
#endif
