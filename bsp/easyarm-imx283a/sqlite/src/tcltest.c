#include "tcl.h"
#include <rtthread.h>
#include "sqlite3.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>

static void init_all(Tcl_Interp *interp){

}

#if defined(FINSH_USING_MSH)
#include <finsh.h>
#include <msh.h>
int cmd_testsql(int argc, char** argv)
{
	int ret = 0;
    Tcl_Interp *interp;

    if (argc < 2)
    {
        rt_kprintf("Usage: testsql test file name\n");
        return -1;
    }

    /* Call sqlite3_shutdown() once before doing anything else. This is to
    ** test that sqlite3_shutdown() can be safely called by a process before
    ** sqlite3_initialize() is. */
    sqlite3_shutdown();

    Tcl_FindExecutable(argv[0]);
    Tcl_SetSystemEncoding(NULL, "utf-8");

    interp = Tcl_CreateInterp();
    init_all(interp);

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
FINSH_FUNCTION_EXPORT_ALIAS(cmd_testsql, __cmd_testsql, Test Sqlite Using TCL)
#endif //FINSH_USING_MSH
