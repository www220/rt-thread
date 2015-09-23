#include <rtthread.h>
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include <dfs_def.h>
#include <dfs_posix.h>
#include <rtdevice.h>

void inittmppath(void)
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
    
    mkdir(rttCfgFileDir, 666);
    mkdir(rttResFileDir, 666);
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

FINSH_FUNCTION_EXPORT_ALIAS(cmd_sh, __cmd_sh, Shell the FILEs.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_msleep, __cmd_msleep, Sleep ms.)
#endif //FINSH_USING_MSH
