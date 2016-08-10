#include <rtthread.h>
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include <dfs_def.h>
#include <dfs_posix.h>
#include <rtdevice.h>

void inittmppath(void)
{
    mkdir(rttCfgFileDir, 666);
    mkdir(rttLogFileDir, 666);
    cleartmppath();
}

void cleartmppath(void)
{
    char buf[100];
	struct dirent *ent = NULL;

	DIR *pDir = opendir(rttTempFileDir);
	if (pDir != NULL)
    {
        while((ent=readdir(pDir)) != NULL)
        {  
			if ((ent->d_type & DFS_DT_DIR) == 0)
            {
				sprintf(buf,"%s/%s",rttTempFileDir,ent->d_name);
				unlink(buf);
			}
		}
		closedir(pDir);
	}
}
RTM_EXPORT(cleartmppath);

char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file)
{
	FILE *pFile = NULL;
	char line[512] = { "" };
	char *find = NULL;
	char *nextp = NULL;
	int pos = 0;

	if (!buf || !buflen)
		return NULL;
	buf[pos] = '\0';
	pFile = fopen(file, "r");
	if (pFile == NULL)
		return buf;

	while (fgets(line, (int) sizeof(line), pFile) != NULL) {
		/* ignore comments */
		if (line[0] == ';')
			continue;
		find = NULL;
		nextp = line;
		while (*nextp) {
			if (*nextp == '"')
			{
				if (find == NULL)
				{
					find = nextp+1;
				}
				else
				{
					*nextp = '\0';
					nextp += 1;
					break;
				}
			}
			nextp++;
		}
		/* not find name */
		if (find == NULL || strcasecmp(name,find) != 0)
			continue;
		find = NULL;
		while (*nextp && buflen) {
			if (*nextp == '"')
			{
				if (find == NULL)
				{
					find = nextp+1;
					nextp++;
					continue;
				}
				else
				{
					*nextp = '\0';
					nextp += 1;
					break;
				}
			}
			if (find == NULL)
			{
				nextp++;
				continue;
			}
			if (*nextp == '\\')
			{
				nextp++;
				if (*nextp == '\0')
					break;
				else if (*nextp == 'r')
					buf[pos++] = '\r';
				else if (*nextp == 'n')
					buf[pos++] = '\n';
				else if (*nextp == 't')
					buf[pos++] = '\t';
				else if (*nextp == '\\')
					buf[pos++] = '\\';
				else if (*nextp == '\"')
					buf[pos++] = '\"';
				else
					buf[pos++] = *nextp;
				buflen--;
			}
			else
			{
				buf[pos++] = *nextp;
				buflen--;
			}
			nextp++;
		}
		/* make buf \0 end char */
		if (buflen == 0)
			buf[pos-1] = '\0';
		else
			buf[pos] = '\0';
		break;
	}
	fclose(pFile);
	return buf;
}
RTM_EXPORT(GetPrivateStringData);

int SetPrivateStringData(const char *name, const char *buf, const char *file)
{
	FILE *pFile = NULL;
	char line[512] = { "" };
	char *find = NULL;
	char *nextp = NULL;
    char *filebuf = NULL;
	int filepos,fileend = -1,filelen;

	pFile = fopen(file, "r+");
	if (pFile == NULL)
    {
        pFile = fopen(file, "w+");
        if (pFile == NULL)
            return 0;
    }

    fseek(pFile,0,SEEK_END);
    filelen = ftell(pFile);
    fseek(pFile,0,SEEK_SET);
    filebuf = (char *)malloc(filelen+10);
    if (filebuf == NULL)
    {
        fclose(pFile);
        return 0;
    }

    filepos = 0;
	while (fgets(line, (int) sizeof(line), pFile) != NULL) {
		/* ignore comments */
		if (line[0] == ';')
        {
            filepos = ftell(pFile);
			continue;
        }
		find = NULL;
		nextp = line;
		while (*nextp) {
			if (*nextp == '"')
			{
				if (find == NULL)
				{
					find = nextp+1;
				}
				else
				{
					*nextp = '\0';
					nextp += 1;
					break;
				}
			}
			nextp++;
		}
		/* not find name */
		if (find == NULL || strcasecmp(name,find) != 0)
        {
            filepos = ftell(pFile);
			continue;
        }
        fileend = ftell(pFile);
        fseek(pFile,0,SEEK_SET);
        fread(filebuf,filepos,1,pFile);
        fseek(pFile,fileend,SEEK_SET);
        fread(filebuf+filepos,filelen-fileend,1,pFile);
		break;
	}
	fclose(pFile);
    //为找到匹配的数据
    if (fileend < 0)
    {
        pFile = fopen(file, "a+");
        if (pFile == NULL)
        {
            free(filebuf);
            return 0;
        }
        fprintf(pFile,"\"%s\"=\"%s\"\n",name,buf);
    }
    else
    {
        pFile = fopen(file, "w+");
        if (pFile == NULL)
        {
            free(filebuf);
            return 0;
        }
        fwrite(filebuf,filepos+filelen-fileend,1,pFile);
        fprintf(pFile,"\"%s\"=\"%s\"\n",name,buf);
    }
    free(filebuf);
    fclose(pFile);
	return 1;
}
RTM_EXPORT(SetPrivateStringData);

#ifdef RT_USING_LWIP
#include "lwip/netif.h"
int getnetif_addr(char* netif_name, u32_t *ip_addr, u32_t *ip_gate, u32_t *ip_mask, u32_t *bs_addr)
{
    unsigned long addr = 0;
    struct netif * netif = netif_list;

    if(strlen(netif_name) > sizeof(netif->name))
        return -1;

    while(netif != RT_NULL)
    {
        if(strncmp(netif_name, netif->name, sizeof(netif->name)) == 0)
            break;

        netif = netif->next;
        if( netif == RT_NULL )
            return -1;
    }

    if (ip_addr)
        *ip_addr = netif->ip_addr.addr;
    if (ip_gate)
        *ip_gate = netif->gw.addr;
    if (ip_mask)
        *ip_mask = netif->netmask.addr;
    addr = ~(netif->netmask.addr);
    if (bs_addr)
        *bs_addr = (netif->ip_addr.addr) | addr;
    return 0;
}
RTM_EXPORT(getnetif_addr);
#endif

#if defined(FINSH_USING_MSH)
#include <shell.h>
#include <finsh.h>
#include <msh.h>
int cmd_sh(int argc, char** argv)
{
	FILE *file;
	int linelen;
	char line[100] = { "" };

    if (argc == 1)
    {
        rt_kprintf("Usage: sh sh file name\n");
        return -1;
    }
    if ((file = fopen(argv[1],"r")) == NULL)
    {
        rt_kprintf("Usage: sh can't open file\n");
        return -1;
    }
	while (fgets(line, (int) sizeof(line), file) != NULL) {
		/* ignore comments */
		if (line[0] == ';' || line[0] == '#'
				|| line[0] == '\n' || line[0] == '\r' || line[0] == 0)
			continue;
		/* skip \r\n */
		linelen = 0;
		while (line[linelen]) {
			if (line[linelen] == '\r' || line[linelen] == '\n') {
				line[linelen] = 0;
				break;
			}
			linelen++;
		}
		/* run sh */
		if (rt_strncmp(line,"waitntp ",8) == 0)
		{
			//wait for ntp
		    int i,usd = atol(line+8);
		    if (usd < 0)
		        usd = 1;
			rt_kprintf("waitntp %d\n",usd);
		    for (i=0; i<usd; i++)
		    {
		        if (time(NULL) < 302400000)
		            rt_thread_delay(1000);
		        else
		            break;
		    }
		}
		else
		{
			rt_kprintf("%s\n",line);
			msh_exec(line,linelen);
			rt_kprintf(FINSH_PROMPT);
		}
	}
    fclose(file);
    return 0;
}
RTM_EXPORT(finsh_get_prompt);
RTM_EXPORT(msh_exec);

int cmd_msleep(int argc, char** argv)
{
	int sleep = 1000;
	if (argc > 1)
		sleep = atol(argv[1]);
	rt_thread_delay(sleep);
	return 0;
}

#ifdef RT_USING_TCLSHELL
#include "tcl.h"
int cmd_tcl(int argc, char** argv)
{
	int ret = 0;
    Tcl_Interp *interp;
    
    if (argc < 2)
    {
        rt_kprintf("Usage: tcl tcl file name\n");
        return -1;
    }
    
    Tcl_FindExecutable(argv[0]);
    Tcl_SetSystemEncoding(NULL, "utf-8");
    
    interp = Tcl_CreateInterp();
    Tcl_SetVar(interp,"argc", "0", TCL_GLOBAL_ONLY);
    Tcl_SetVar(interp,"argv0",argv[1],TCL_GLOBAL_ONLY);
    Tcl_SetVar(interp,"argv", "", TCL_GLOBAL_ONLY);
    
    ret = Tcl_EvalFile(interp, argv[1]);
    if (ret !=TCL_OK)
    {
        const char *zInfo = Tcl_GetVar(interp, "errorInfo", TCL_GLOBAL_ONLY);
        if( zInfo==0 ) 
            zInfo = Tcl_GetStringResult(interp);
        rt_kprintf("%s: %s\n", *argv, zInfo);
    }

    Tcl_DeleteInterp(interp);
    return ret;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_tcl, __cmd_tcl, TCL.)
#endif

extern int dfs_mount(const char *device_name, const char *path, const char *filesystemtype, rt_uint32_t rwflag, const void *data);
int cmd_mount(int argc, char **argv)
{
    if (argc < 4)
    {
        rt_kprintf("Usage: mount ftl0 /spi elm\n");
        return -RT_ERROR;
    }
    return dfs_mount(argv[1], argv[2], argv[3], 0, 0);
}

extern int dfs_unmount(const char *specialfile);
int cmd_unmount(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: unmount /spi\n");
        return -RT_ERROR;
    }
    return dfs_unmount(argv[1]);
}

extern int df(const char *path);
int cmd_df(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: df /spi\n");
        return -RT_ERROR;
    }
    return df(argv[1]);
}

int cmd_uptime(int argc, char** argv)
{
    unsigned updays, uphours, upminutes;

    updays = (unsigned) uptime_count / (unsigned)(60*60*24);
    if (updays)
        rt_kprintf("%u day%s, ", updays, (updays != 1) ? "s" : "");
    upminutes = (unsigned) uptime_count / (unsigned)60;
    uphours = (upminutes / (unsigned)60) % (unsigned)24;
    upminutes %= 60;
    if (uphours)
        rt_kprintf("%2u:%02u\n", uphours, upminutes);
    else
        rt_kprintf("%u min\n", upminutes);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_uptime, __cmd_uptime, system uptime);

char *getsave_loginfo(const char* file, int length)
{
    int fd,hr,filelength = 0;
    char *buffer,*find;

    if (length < 128)
        length = 128;

    if ((fd = open(file, O_RDONLY)) < 0)
        return NULL;

    buffer = (char *)malloc(length+1);
    if (buffer == RT_NULL)
    {
        close(fd);
        return NULL;
    }

    filelength = lseek(fd, 0, SEEK_END);
    if (filelength > length)
        filelength = filelength-length;
    else
        filelength = 0;
    lseek(fd, filelength, SEEK_SET);
    length = read(fd, buffer, length);
    buffer[length] = 0;
    close(fd);

    hr = 0;
    find = buffer;
    while (*find  != '\0')
    {
        if (*find != '\r' && *find != '\n')
        {
            if (hr == 0)
                hr = 1;
        }
        else if (hr == 1)
        {
            while (*find == '\r' || *find == '\n') find++;
            break;
        }
        find++;
    }
    if (*find)
    {
        find = rt_strdup(find);
        free(buffer);
        return find;
    }
    return buffer;
}
RTM_EXPORT(getsave_loginfo);

int cmd_tail(int argc, char **argv)
{
    char *buffer;
    rt_uint32_t length = 2048;

    if (argc == 1)
    {
        rt_kprintf("Usage: tail [FILE]...\n");
        rt_kprintf("Tail FILE(s)\n");
        return 0;
    }

    if (argc > 2)
        length = atol(argv[2]);

    buffer = getsave_loginfo(argv[1], length);
    if (buffer == NULL)
    {
        rt_kprintf("Open %s failed\n", argv[1]);
        return 1;
    }

    rt_kprintf("%s", buffer);
    free(buffer);
    return 0;
}

#ifdef RT_USING_RYM
#include <ymodem.h>

struct custom_ctx
{
    struct rym_ctx parent;
    int fd,flen;
    char fpath[100];
};

static enum rym_code _rym_bg(
        struct rym_ctx *ctx,
        rt_uint8_t *buf,
        rt_size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx*)ctx;
    /* the buf should be the file name */
    sprintf(cctx->fpath, "/mmc/%s", (const char*)buf);
    cctx->fd = open(cctx->fpath, O_CREAT | O_WRONLY | O_TRUNC, 0);
    if (cctx->fd < 0)
    {
        rt_err_t err = rt_get_errno();
        rt_kprintf("error creating file: %d\n", err);
        rt_kprintf("abort transmission\n");
        return RYM_CODE_CAN;
    }

    cctx->flen = atoi((const char*)buf+strlen((const char*)buf)+1);
    if (cctx->flen == 0)
        cctx->flen = -1;
    return RYM_CODE_ACK;
}

static enum rym_code _rym_tof(
        struct rym_ctx *ctx,
        rt_uint8_t *buf,
        rt_size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx*)ctx;
    RT_ASSERT(cctx->fd >= 0);
    if (cctx->flen == -1)
    {
        write(cctx->fd, buf, len);
    }
    else
    {
        int wlen = len > cctx->flen ? cctx->flen : len;
        write(cctx->fd, buf, wlen);
        cctx->flen -= wlen;
    }
    return RYM_CODE_ACK;
}

static enum rym_code _rym_end(
        struct rym_ctx *ctx,
        rt_uint8_t *buf,
        rt_size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx*)ctx;

    RT_ASSERT(cctx->fd >= 0);
    close(cctx->fd);
    cctx->fd = -1;

    return RYM_CODE_ACK;
}

int cmd_ym(int argc, char **argv)
{
    int ret;
    struct custom_ctx ym;
    rt_device_t dev = rt_console_get_device();
    rt_uint16_t oflag = dev->open_flag;

    rt_kprintf("entering RYM mode\n");
    ret = rym_recv_on_device(&ym.parent,dev,RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX,
                                _rym_bg,_rym_tof,_rym_end,10);
    rt_thread_delay(500);
    dev->open_flag = oflag;
    if (ret != 0)
        rt_kprintf("\nleaving RYM mode with code %d\n", ret);
    rt_kprintf("file %s has been created.\n", ym.fpath);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_ym, __cmd_ym, ymodem recv file);
#endif

FINSH_FUNCTION_EXPORT_ALIAS(cmd_df, __cmd_df, get disk free);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_unmount, __cmd_unmount, unmount path);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mount, __cmd_mount, mount path to device);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sh, __cmd_sh, Shell the FILEs.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_msleep, __cmd_msleep, Sleep ms.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_tail, __cmd_tail, Tail FILE(s));
#endif //FINSH_USING_MSH
