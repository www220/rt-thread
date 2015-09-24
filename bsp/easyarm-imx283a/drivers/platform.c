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
    mkdir(rttResFileDir, 666);
    cleartmppath();
}

void cleartmppath(void)
{
#ifdef RT_USING_DFS
    char buf[100];
	struct dirent *ent = NULL;

	DIR *pDir = opendir(rttTempFileDir);
	if (pDir == NULL)
    {
		mkdir(rttTempFileDir,0);
	}
    else
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
#endif
}

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
RTM_EXPORT(GetPrivateStringData)

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
		rt_kprintf("%s\n",line);
		msh_exec(line,linelen);
		rt_kprintf(FINSH_PROMPT);
	}
    fclose(file);
    return 0;
}

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

int cmd_echo(int argc, char** argv)
{
    struct dfs_fd fd;
    if (argc < 3)
    {
        rt_kprintf("Usage: echo [FILE] [INFO]\n");
        rt_kprintf("Write FILE...\n");
        return 0;
    }

    if (dfs_file_open(&fd, argv[1], DFS_O_CREAT|DFS_O_RDWR) < 0)
    {
        rt_kprintf("Open %s failed\n", argv[1]);
        return 1;
    }
    dfs_file_write(&fd,argv[2],strlen(argv[2]));
    dfs_file_write(&fd,"\r\n",2);
    dfs_file_close(&fd);
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_echo, __cmd_echo, Write FILE);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_df, __cmd_df, get disk free);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_unmount, __cmd_unmount, unmount path);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mount, __cmd_mount, mount path to device);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sh, __cmd_sh, Shell the FILEs.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_msleep, __cmd_msleep, Sleep ms.)
#endif //FINSH_USING_MSH
