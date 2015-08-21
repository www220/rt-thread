#include <rtthread.h>
#include "board.h"
#include <stdio.h>
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
#endif
}

void cpu_usage_idle_hook(void)
{
    wtdog_count = 0;
}


