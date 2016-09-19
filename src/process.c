/*
 * File      : process.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-19     Www220       first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtm.h>
#include <mmu.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/stat.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#include <shell.h>
#endif

#ifdef RT_USING_DFS
#include <dfs.h>
#include <dfs_def.h>
#endif

#ifdef RT_USING_PROCESS
#include "module.h"
#include "board.h"
#include "linux-usedef.h"

#define elf_module        ((Elf32_Ehdr *)module_ptr)
#define shdr              ((Elf32_Shdr *)((rt_uint8_t *)module_ptr + elf_module->e_shoff))
#define phdr              ((Elf32_Phdr *)((rt_uint8_t *)module_ptr + elf_module->e_phoff))

#define IS_PROG(s)        (s.sh_type == SHT_PROGBITS)
#define IS_NOPROG(s)      (s.sh_type == SHT_NOBITS)
#define IS_REL(s)         (s.sh_type == SHT_REL)
#define IS_RELA(s)        (s.sh_type == SHT_RELA)
#define IS_ALLOC(s)       (s.sh_flags == SHF_ALLOC)
#define IS_AX(s)          ((s.sh_flags & SHF_ALLOC) && (s.sh_flags & SHF_EXECINSTR))
#define IS_AW(s)          ((s.sh_flags & SHF_ALLOC) && (s.sh_flags & SHF_WRITE))

#ifndef RT_DEBUG_PROCESS
#define RT_DEBUG_PROCESS 0
#endif

#ifndef RT_USING_PROCESS_STKSZ
#define RT_USING_PROCESS_STKSZ (4096 * 32)
#endif

#ifndef RT_USING_PROCESS_PRIO
#define RT_USING_PROCESS_PRIO (RT_THREAD_PRIORITY_MAX - 4)
#endif

#if RT_MM_PAGE_SIZE != 4096
#error "RT_MM_PAGE_SIZE == 4096"
#endif
#ifndef RT_USING_SLAB
#error "defined RT_USING_SLAB"
#endif
#if PROCESS_MAX > 60
#error "PROCESS_MAX <= 60"
#endif
#if PROCESS_MEM > 64
#error "PROCESS_MEM <= 64"
#endif

#define PAGE_COUNT_MAX    (256 * PROCESS_MEM)
#define RT_PROCESS_ARG_MAX    20
#define RT_PROCESS_MEMMAKE (32 * 1024)
extern rt_thread_t rt_thread_create2(const char *name,
                             void (*entry)(void *parameter),
                             void       *parameter,
                             void       *stack_start,
                             rt_uint32_t stack_size,
                             rt_uint8_t  priority,
                             rt_uint32_t tick);
extern int __rt_ffs(int value);
extern rt_uint32_t rt_pids_from;
extern rt_uint32_t rt_pids_to;

//status
volatile int pidbuf[MAX_PID_SIZE];
//pz:[0],ppid:[1],pgid[2],sid[3]
volatile unsigned short pidinfo[MAX_PID_SIZE][4];
static int current_tpid = 0;
static int sum_tpid = 0;
//pid
static volatile rt_uint32_t pidid[1+PROCESS_MAX/32];
static int current_pid = 0;
//wait
static rt_uint32_t mod_waitp;
static struct rt_event mod_eventp;
//kill
int rt_process_kill(rt_process_t module, int pid, int sig);

static unsigned short getempty_pid()
{
	int i,ret;
	for(i=0; i<PROCESS_MAX; i++)
	{
		if ((pidid[current_pid/32] & (1<<(i%32))) == 0)
		{
			ret = current_pid+1;
			pidid[current_pid/32] |= (1<<(i%32));
			if (++current_pid >= PROCESS_MAX)
				current_pid = 0;
			return ret;
		}
	}
	return 0;
}
static void free_pid(unsigned short pid)
{
	if (pid <= 0)
		return;
	--pid;
	pidid[pid/32] &= ~(1<<(pid%32));
}

static unsigned short getempty_tpid(unsigned short parent)
{
	int i,ret;
	for(i=0; i<MAX_PID_SIZE; i++)
	{
		if (pidinfo[current_tpid][0] == 0)
		{
			ret = current_tpid+1;
			pidbuf[current_tpid] = 0;
			pidinfo[current_tpid][0] = 101;
			pidinfo[current_tpid][1] = parent;
			pidinfo[current_tpid][2] = parent;
			pidinfo[current_tpid][3] = parent;
			if (parent)
			{
				pidinfo[current_tpid][2] = pidinfo[parent-1][2];
				pidinfo[current_tpid][3] = pidinfo[parent-1][3];
			}
			if (++current_tpid >= MAX_PID_SIZE)
				current_tpid = 0;
			sum_tpid++;
			return ret;
		}
	}
	return 0;
}
static void free_tpid(unsigned short tpid)
{
	if (tpid <= 0)
		return;
	pidinfo[tpid-1][0] = 0;
	pidbuf[tpid-1] = 0;
	//没有进程的时候清理
	if (--sum_tpid == 0)
	{
		RT_ASSERT(tpid == 1);
		current_tpid = 0;
	}
}
static void release_tpid(unsigned short tpid, int exitcode)
{
	int i;
	if (tpid <= 0)
		return;
	pidinfo[tpid-1][0] = 102;
	pidbuf[tpid-1] = exitcode;
	if (tpid == 1)
	{
		RT_ASSERT(sum_tpid == 1);
		rt_event_send(&mod_eventp, 1);
		return;
	}
	//托付孤儿进程
	for(i=0; i<MAX_PID_SIZE; i++)
	{
		if (pidinfo[i][0] < 100)
			continue;
		if (pidinfo[i][1] == tpid)
			pidinfo[i][1] = 1;
	}
	rt_event_send(&mod_eventp, 0xfffffffe);
}
static int find_tpid(unsigned short tpid, unsigned short tme)
{
	int i,ret;
	for(i=0; i<MAX_PID_SIZE; i++)
	{
		if (pidinfo[i][0] == 0)
			continue;
		if (i+1 == tme)
			continue;
		if ((pidinfo[i][1] == tpid)
				|| (pidinfo[i][2] == tpid)
				|| (pidinfo[i][3] == tpid))
			return 1;
	}
	return 0;
}

static void rt_process_timeout(void *parameter)
{
    struct rt_process *process = (struct rt_process *)parameter;

    RT_ASSERT(process != RT_NULL);
    rt_process_kill(process, process->tpid, SIGALRM);
}

//以下函数实现系统对象的复制
extern struct rt_object_information rt_object_container[];
rt_object_t rt_process_copy_object(rt_object_t desc, rt_object_t src)
{
	int i,type = src->type&0x7f;
	register rt_base_t temp;
	struct rt_object_information *information;
	if (desc == RT_NULL)
	{
		desc = rt_object_allocate(type,"");
	}
	else
	{
		temp = rt_hw_interrupt_disable();
		/* insert object into information object list */
		rt_list_insert_after(&(src->list), &(desc->list));
		rt_hw_interrupt_enable(temp);
	}
	if (desc == RT_NULL)
		return desc;

	information = &rt_object_container[type];
	rt_enter_critical();
	rt_memcpy(desc,src,information->object_size);

	/* dump object */
	switch(type)
	{
	case RT_Object_Class_Timer:
	{
		rt_timer_t src_timer = (rt_timer_t)src;
		rt_timer_t desc_timer = (rt_timer_t)desc;
		for (i = 0; i < RT_TIMER_SKIP_LIST_LEVEL; i++)
		{
			rt_list_init(&(desc_timer->row[i]));
			if (!rt_list_isempty(&(src_timer->row[i])))
				rt_list_insert_after(&(src_timer->row[i]), &(desc_timer->row[i]));
		}
		//fix parameter
		break;
	}
	case RT_Object_Class_Thread:
	{
		rt_thread_t src_thread = (rt_thread_t)src;
		rt_thread_t desc_thread = (rt_thread_t)desc;
		/* remove thread from run list */
		rt_list_init(&(desc_thread->tlist));
		//timer
		rt_process_copy_object(&desc_thread->thread_timer.parent, &src_thread->thread_timer.parent);
		desc_thread->thread_timer.parameter = desc_thread;
		//fix process_id
		break;
	}
	}

	rt_exit_critical();
	return desc;
}

/**
 * @ingroup SystemInit
 *
 * This function will initialize system process
 */
int rt_system_process_init(void)
{
	mod_waitp = 0xfffffffe;
	rt_event_init(&mod_eventp, "process", RT_IPC_FLAG_FIFO);
	return 0;
}
INIT_COMPONENT_EXPORT(rt_system_process_init);

/**
 * This function will return self process object
 *
 * @return the self process object
 */
rt_process_t rt_process_self(void)
{
    rt_thread_t tid;

    tid = rt_thread_self();
    if (tid == RT_NULL)
        return RT_NULL;

    /* return current process */
    return (rt_process_t)tid->process_id;
}
RTM_EXPORT(rt_process_self);

void rt_process_init_object_container(struct rt_process *module)
{
    RT_ASSERT(module != RT_NULL);

    /* initialize object container - thread */
    rt_list_init(&(module->thread_list));

    /* initialisz file */
    rt_memset(module->file_list, 0, sizeof(module->file_list));
    if (module->tpid == 1)
    {
    	//对于init程序要打开标准的三个文件流
        module->file_list[0] = 1;
        module->file_list[1] = 2;
        module->file_list[2] = 3;
        fd_get(0);fd_get(1);fd_get(2);
    }
}

#ifdef RT_USING_HOOK
static void (*rt_process_load_hook)(rt_process_t process);
static void (*rt_process_unload_hook)(rt_process_t process);

/**
 * @addtogroup Hook
 */

/*@{*/

/**
 * This function will set a hook function, which will be invoked when process
 * be loaded to system.
 *
 * @param hook the hook function
 */
void rt_process_load_sethook(void (*hook)(rt_process_t process))
{
    rt_process_load_hook = hook;
}

/**
 * This function will set a hook function, which will be invoked when process
 * be unloaded from system.
 *
 * @param hook the hook function
 */
void rt_process_unload_sethook(void (*hook)(rt_process_t process))
{
    rt_process_unload_hook = hook;
}

/*@}*/
#endif

static struct rt_process *_load_exec_object(rt_process_t process,
                                             const char *name,
                                             void      *module_ptr)
{
    rt_process_t module = RT_NULL;
    rt_uint32_t index, module_size = 0;
    Elf32_Addr vstart_addr, vend_addr;
    rt_bool_t has_vstart;

    RT_ASSERT(module_ptr != RT_NULL);

    /* get the ELF image size */
    has_vstart = RT_FALSE;
    vstart_addr = vend_addr = RT_NULL;
    for (index = 0; index < elf_module->e_phnum; index++)
    {
        if (phdr[index].p_type != PT_LOAD)
            continue;

        RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("LOAD segment: %d, 0x%p, 0x%08x\n",
                                       index, phdr[index].p_vaddr, phdr[index].p_memsz));

        if (phdr[index].p_memsz < phdr[index].p_filesz)
        {
            rt_kprintf("invalid elf: segment %d: p_memsz: %d, p_filesz: %d\n",
                       index, phdr[index].p_memsz, phdr[index].p_filesz);
            return RT_NULL;
        }
        if (!has_vstart)
        {
            vstart_addr = phdr[index].p_vaddr;
            vend_addr = phdr[index].p_vaddr + phdr[index].p_memsz;
            has_vstart = RT_TRUE;
            if (vend_addr < vstart_addr || vstart_addr != PROCESS_BASE)
            {
                rt_kprintf("invalid elf: segment %d: p_vaddr: %d, p_memsz: %d\n",
                           index, phdr[index].p_vaddr, phdr[index].p_memsz);
                return RT_NULL;
            }
        }
        else
        {
            /* There should not be too much padding in the object files. */
            rt_kprintf("warning: too much padding before segment %d\n", index);
            return RT_NULL;
        }
    }

    module_size = vend_addr - vstart_addr;
    module_size = RT_ALIGN(module_size,32*RT_MM_PAGE_SIZE);

    RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("process size: %d, vstart_addr: 0x%p\n",
                                   module_size, vstart_addr));

    if (module_size == 0)
    {
        rt_kprintf("Process: size error\n");

        return RT_NULL;
    }

    /* allocate process */
    module = (struct rt_process *)rt_object_allocate(RT_Object_Class_Process,
                                                    name);
    if (!module)
        return RT_NULL;

    module->vstart_addr = vstart_addr;
    module->module_size = module_size;

    module->page_array = RT_NULL;
    module->page_cnt = 0;
    module->page_mutex = RT_NULL;
    module->alarm = RT_NULL;
    module->impure_ptr = RT_NULL;
    module->jmppid = 0;
    module->jmpsp = 0;
    module->jmpsplen = 0;
    module->exitcode = 0;

    {register rt_ubase_t temp = rt_hw_interrupt_disable();
    module->pid = getempty_pid();
    if (module->pid)
    {
        rt_process_t parent = rt_process_self();
        module->tpid = (process)?(process->tpid):(getempty_tpid(parent?parent->tpid:0));
        if (module->tpid == 0)
        {
            free_pid(module->pid);
            module->pid = 0;
        }
    }
    rt_hw_interrupt_enable(temp);}
    if (module->pid == 0)
    {
        rt_kprintf("Process: allocate pid failed.\n");
        rt_object_delete(&(module->parent));

        return RT_NULL;
    }

    /* allocate process space */
    module->module_space = rt_page_alloc(module_size/RT_MM_PAGE_SIZE);
    if (module->module_space == RT_NULL)
    {
        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(module->pid);
        free_tpid(module->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate space failed.\n");
        rt_object_delete(&(module->parent));

        return RT_NULL;
    }

    /* zero all space */
    mmu_maketlb(module->pid,0);
    rt_memset(module->module_space, 0, module_size);
    mmu_usermap(module->pid,(Elf32_Addr)module->module_space,vstart_addr,module_size,0);

    for (index = 0; index < elf_module->e_phnum; index++)
    {
        if (phdr[index].p_type == PT_LOAD)
        {
            rt_memcpy(module->module_space + phdr[index].p_vaddr - vstart_addr,
                      (rt_uint8_t *)elf_module + phdr[index].p_offset,
                      phdr[index].p_filesz);
        }
    }

    /* set process entry */
    module->module_entry = (void *)elf_module->e_entry;

    /* construct process symbol table */
    for (index = 0; index < elf_module->e_shnum; index ++)
    {
        /* find .symtab section */
        rt_uint8_t *shstrab;
        shstrab = (rt_uint8_t *)module_ptr +
            shdr[elf_module->e_shstrndx].sh_offset;
        if (rt_strcmp((const char *)(shstrab + shdr[index].sh_name), ELF_SYMTAB) == 0)
            break;
    }

    /* found .symtab section */
    if (index != elf_module->e_shnum)
    {
        int i, count = 0;
        Elf32_Sym  *symtab = RT_NULL;
        rt_uint8_t *strtab = RT_NULL;

        symtab =(Elf32_Sym *)((rt_uint8_t *)module_ptr + shdr[index].sh_offset);
        strtab = (rt_uint8_t *)module_ptr + shdr[shdr[index].sh_link].sh_offset;

        for (i = 0, count = 0; i < shdr[index].sh_size/sizeof(Elf32_Sym); i++)
        {
            if ((ELF_ST_BIND(symtab[i].st_info) != STB_GLOBAL) ||
                (ELF_ST_TYPE(symtab[i].st_info) != STT_OBJECT))
                continue;

            if (rt_strcmp((const char *)(strtab + symtab[i].st_name), "_impure_ptr") == 0)
            {
                struct _reent **impure_ptr = (struct _reent **)(module->module_space+symtab[i].st_value-vstart_addr);
                module->impure_ptr = *impure_ptr;
            }
        }
    }

    return module;
}

SECTION(".process_fn")
static int _rt_process_split_arg(char* cmd, rt_size_t length, char* argv[])
{
    int argc = 0;
    char *ptr = cmd;

    while ((ptr - cmd) < length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && (ptr -cmd)< length)
            *ptr++ = '\0';
        /* check whether it's the end of line */
        if ((ptr - cmd)>= length) break;

        /* handle string with quote */
        if (*ptr == '"')
        {
            argv[argc++] = ++ptr;

            /* skip this string */
            while (*ptr != '"' && (ptr-cmd) < length)
                if (*ptr ++ == '\\')  ptr ++;
            if ((ptr - cmd) >= length) break;

            /* skip '"' */
            *ptr ++ = '\0';
        }
        else
        {
            argv[argc++] = ptr;
            while ((*ptr != ' ' && *ptr != '\t') && (ptr - cmd) < length)
                ptr ++;
        }

        if (argc >= RT_PROCESS_ARG_MAX) break;
    }

    return argc;
}

struct process_main_info
{
    rt_uint8_t*                  module_cmd_line;       /**< process command line */
    rt_uint32_t                  module_cmd_size;       /**< the size of process command line */
    rt_uint8_t*                  module_env_line;       /**< process env line */
    rt_uint32_t                  module_env_size;       /**< the size of process env line */
    void                        *module_entry;          /**< the entry address of process */
};

/* process main thread entry */
SECTION(".process_fn")
static void process_main_entry(void* parameter)
{
    int argc,argc2;
    char *argv[RT_PROCESS_ARG_MAX];
    typedef int (*main_func_t)(int argc, char** argv);

    struct process_main_info *module = (struct process_main_info*)parameter;
    if (module == RT_NULL)
        return;

    if (module->module_cmd_line == RT_NULL && module->module_cmd_size != 0)
        /* malloc for process_cmd_line failed. */
        return;

    /* FIXME: we should run some C++ initialize code before jump into the
     * entry. */

    if (module->module_cmd_line == RT_NULL)
    {
        RT_DEBUG_LOG(0, ("run bare entry: 0x%p\n",
                                       module->module_entry));
        ((main_func_t)module->module_entry)(0, RT_NULL);
        return;
    }

    argc = _rt_process_split_arg((char*)module->module_cmd_line,
                                module->module_cmd_size, argv+1);
    if (argc == 0)
        return;

    argc2 = _rt_process_split_arg((char*)module->module_env_line,
                                module->module_env_size, argv+argc+2);

    RT_DEBUG_LOG(0, ("run main entry: 0x%p with %s env %s\n",
                                   module->module_entry,
                                   module->module_cmd_line,
                                   module->module_env_line));
    /* do the main function */
    argv[0] = (char *)argc;
    argv[argc+1] = NULL;
    argv[argc+2+argc2] = NULL;
    ((main_func_t)module->module_entry)(argc, argv);
    return;
}

/**
 * This function will load a process with a main function from memory and create a 
 * main thread for it
 *
 * @param name the name of process, which shall be unique
 * @param module_ptr the memory address of process image
 * @argc the count of argument
 * @argd the argument data, which should be a 
 *
 * @return the process object
 */
rt_process_t rt_process_do_main(rt_process_t process,
                              const char *name,
                              void *module_ptr,
                              const char** argv,
                              const char** envp)
{
    int i;
    rt_process_t module;

    RT_DEBUG_NOT_IN_INTERRUPT;

    RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("rt_process_load: %s\n", name));

    /* check ELF header */
    if (rt_memcmp(elf_module->e_ident, RTMMAG, SELFMAG) != 0 &&
        rt_memcmp(elf_module->e_ident, ELFMAG, SELFMAG) != 0)
    {
        rt_kprintf("Process: magic error\n");

        return RT_NULL;
    }

    /* check ELF class */
    if (elf_module->e_ident[EI_CLASS] != ELFCLASS32)
    {
        rt_kprintf("Process: ELF class error\n");

        return RT_NULL;
    }

    if (elf_module->e_type == ET_EXEC && elf_module->e_entry != 0)
    {
        module = _load_exec_object(process, name, module_ptr);
    }
    else
    {
        rt_kprintf("Process: unsupported elf type\n");

        return RT_NULL;
    }

    if (module == RT_NULL)
        return RT_NULL;

    /* init process object container */
    rt_process_init_object_container(module);
    if (process)
        rt_memcpy(module->workd, process->workd, 256);
    else
        rt_strncpy(module->workd, "/", 256);
    for (i=0; i<NSIG; i++)
    {
        module->sigact[i].sa_handler = SIG_DFL;
        sigemptyset(&module->sigact[i].sa_mask);
        module->sigact[i].sa_flags = 0;
    }
    sigemptyset(&module->sigset);
    sigemptyset(&module->siginfo);

    if (*argv && (*argv)[0])
    {
        /* set process argument */
        extern char **environ;
        struct process_main_info main_info;
        int line_size = process?0:rt_strlen(*argv);
        const char **arg = argv;
        while (process && *arg)
        {
            line_size += ((arg != argv)?1:0)+rt_strlen(*arg);
            arg++;
        }
        int env_size = 0;
        const char **env = envp?envp:(const char **)environ;
        while (*env)
        {
            env_size += ((env != (envp?envp:(const char **)environ))?3:2)+rt_strlen(*env);
            env++;
        }
        module->module_cmd_size = RT_ALIGN(RT_PROCESS_MEMMAKE+RT_USING_PROCESS_STKSZ,RT_MM_PAGE_SIZE);
        module->module_cmd_line = (rt_uint8_t*)rt_page_alloc(module->module_cmd_size/RT_MM_PAGE_SIZE);
        if (module->module_cmd_line)
        {
            main_info.module_cmd_line = module->module_cmd_line+sizeof(main_info);
            main_info.module_cmd_size = line_size;
            main_info.module_env_line = main_info.module_cmd_line+main_info.module_cmd_size+1;
            main_info.module_env_size = env_size;
            main_info.module_entry = module->module_entry;

            rt_memcpy(module->module_cmd_line, &main_info, sizeof(main_info));
            if (process){
            arg = argv;
            line_size = 0;
            while (*arg)
            {
                int strlen = rt_strlen(*arg);
                if(arg != argv)
                    main_info.module_cmd_line[line_size++] = ' ';
                rt_memcpy(&main_info.module_cmd_line[line_size], *arg, strlen);line_size += strlen;
                arg++;
            }
            main_info.module_cmd_line[line_size] = '\0';}
            else
            {
                rt_memcpy(main_info.module_cmd_line, *argv, line_size);
                main_info.module_cmd_line[line_size] = '\0';
            }

            env = envp?envp:(const char **)environ;
            env_size = 0;
            while (*env)
            {
                int strlen = rt_strlen(*env);
                if(env != (envp?envp:(const char **)environ))
                    main_info.module_env_line[env_size++] = ' ';
                main_info.module_env_line[env_size++] = '"';
                rt_memcpy(&main_info.module_env_line[env_size], *env, strlen);env_size += strlen;
                main_info.module_env_line[env_size++] = '"';
                env++;
            }
            main_info.module_env_line[env_size] = '\0';

            RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("argv : %s \n",main_info.module_cmd_line));
            RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("envp : %s \n",main_info.module_env_line));

            mmu_usermap(module->pid,(rt_uint32_t)module->module_cmd_line,
                    module->vstart_addr-module->module_cmd_size,module->module_cmd_size,0);
            /* hold thread stack */
            mmu_userunmap(module->pid,module->vstart_addr-RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);
            mmu_userunmap(module->pid,module->vstart_addr-RT_USING_PROCESS_STKSZ-2*RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);
        }
        else
        {
            /* release process space memory */
            rt_page_free(module->module_space,module->module_size/RT_MM_PAGE_SIZE);

            {register rt_ubase_t temp = rt_hw_interrupt_disable();
            free_pid(module->pid);
            free_tpid(module->tpid);
            rt_hw_interrupt_enable(temp);}

            rt_kprintf("Process: allocate cmd buffer failed.\n");
            rt_object_delete(&(module->parent));

            return RT_NULL;
        }
    }
    else
    {
        /* initialize an empty command */
        module->module_cmd_line = RT_NULL;
        module->module_cmd_size = 0;
    }

#ifdef RT_USING_SLAB
    /* create page array */
    module->page_array = (void *)rt_malloc(PAGE_COUNT_MAX*sizeof(rt_uint32_t));
    module->page_cnt = 0;

    /* initialize heap semaphore */
    module->page_mutex = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
    module->alarm = rt_timer_create(name, rt_process_timeout, module, 0, RT_TIMER_FLAG_ONE_SHOT);

    if (!module->page_array || !module->page_mutex|| !module->alarm)
    {
        /* release process space memory */
        rt_page_free(module->module_space,module->module_size/RT_MM_PAGE_SIZE);

        /* delete command line */
        rt_page_free(module->module_cmd_line,module->module_cmd_size/RT_MM_PAGE_SIZE);

        if (module->page_array != RT_NULL)
            rt_free(module->page_array);
        if (module->page_mutex != RT_NULL)
            rt_mutex_delete(module->page_mutex);
        if (module->alarm != RT_NULL)
            rt_timer_delete(module->alarm);

        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(module->pid);
        free_tpid(module->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate mem manager failed.\n");
        rt_object_delete(&(module->parent));

        return RT_NULL;
    }
#endif

    /* create process thread */
    module->module_thread = rt_thread_create2(name,
                                             process_main_entry,
                                             (void *)(module->vstart_addr-RT_PROCESS_MEMMAKE-RT_USING_PROCESS_STKSZ),
                                             module->module_cmd_line+RT_PROCESS_MEMMAKE-RT_MM_PAGE_SIZE,
                                             RT_USING_PROCESS_STKSZ,
                                             RT_USING_PROCESS_PRIO, 20);
    if (!module->module_thread)
    {
        /* release process space memory */
        rt_page_free(module->module_space,module->module_size/RT_MM_PAGE_SIZE);

        /* delete command line */
        rt_page_free(module->module_cmd_line,module->module_cmd_size/RT_MM_PAGE_SIZE);

        if (module->page_array != RT_NULL)
            rt_free(module->page_array);
        if (module->page_mutex != RT_NULL)
            rt_mutex_delete(module->page_mutex);
        if (module->alarm != RT_NULL)
            rt_timer_delete(module->alarm);

        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(module->pid);
        free_tpid(module->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate thread failed.\n");
        rt_object_delete(&(module->parent));

        return RT_NULL;
    }

    /* set process id */
    module->module_thread->process_id = (void *)module;
    module->module_thread->sp = (void *)module->vstart_addr-(module->module_thread->stack_addr+module->module_thread->stack_size-module->module_thread->sp)-RT_MM_PAGE_SIZE;
    module->module_thread->stack_addr = (void *)module->vstart_addr-RT_USING_PROCESS_STKSZ-RT_MM_PAGE_SIZE;
    module->module_thread->plib_reent = module->impure_ptr;
    struct process_main_info *main_info = (struct process_main_info*)module->module_cmd_line;
    main_info->module_cmd_line = module->module_thread->parameter+sizeof(struct process_main_info);
    main_info->module_env_line = main_info->module_cmd_line+main_info->module_cmd_size+1;

    /* dump open file */
	for (i=0; process && i<DFS_FD_MAX; i++)
	{
		if (process->file_list[i] != 0)
		{
			int fileno = process->file_list[i]-1;
			struct dfs_fd * fd = fd_get(fileno);
			if (fd == RT_NULL)
				continue;
			if (fd->flags & O_CLOEXEC)
			{
				fd_put(fd);
				continue;
			}
			module->file_list[i] = process->file_list[i];
		}
	}

    /* startup process thread */
    rt_thread_startup(module->module_thread);

#ifdef RT_USING_HOOK
    if (rt_process_load_hook != RT_NULL)
    {
    	rt_process_load_hook(module);
    }
#endif

    return module;
}

#ifdef RT_USING_DFS
#include <dfs_posix.h>

static char* _process_name(const char *path)
{
    const char *first, *end, *ptr;
    char *name;
    int size;

    ptr   = (char *)path;
    first = ptr;
    end   = path + rt_strlen(path);

    while (*ptr != '\0')
    {
        if (*ptr == '/')
            first = ptr + 1;
        if (*ptr == '.')
            end = ptr - 1;

        ptr ++;
    }

    size = end - first + 1;
    name = rt_malloc(size);
    rt_strncpy(name, first, size);
    name[size] = '\0';

    return name;
}

/**
 * This function will do a excutable program with main function and parameters.
 *
 * @param path the full path of application process
 * @param cmd_line the command line of program
 * @param size the size of command line of program
 *
 * @return the process object
 */
rt_process_t rt_process_exec_env(rt_process_t process, const char *path, const char** argv, const char **envp)
{
    int fd, length;
    struct rt_process *module;
    struct stat s;
    char *buffer, *offset_ptr;
    char *name;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameters */
    RT_ASSERT(path != RT_NULL);

    if (stat(path, &s) !=0)
    {
        rt_kprintf("Process: access %s failed\n", path);

        return RT_NULL;
    }
    offset_ptr = buffer = (char *)rt_malloc(s.st_size);
    if (buffer == RT_NULL)
    {
        rt_kprintf("Process: out of memory\n");

        return RT_NULL;
    }

    fd = open(path, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("Process: open %s failed\n", path);
        rt_free(buffer);

        return RT_NULL;
    }

    do
    {
        length = read(fd, offset_ptr, 4096);
        if (length > 0)
        {
            offset_ptr += length;
        }
    }while (length > 0);

    /* close fd */
    close(fd);

    if ((rt_uint32_t)offset_ptr - (rt_uint32_t)buffer != s.st_size)
    {
        rt_kprintf("Process: read file failed\n");
        rt_free(buffer);

        return RT_NULL;
    }

    name   = _process_name(path);
    module = rt_process_do_main(process, name, (void *)buffer, argv, envp);
    rt_free(buffer);
    rt_free(name);

    //启动程序以后终止msh的输入
    if (module != RT_NULL && process == RT_NULL)
        tty_rx_inxpz = 0;

    return module;
}

rt_process_t rt_process_exec_cmd(const char *path, const char* cmd_line, int size)
{
	if (size == -1)
		size = rt_strlen(cmd_line);
	if (size == 0)
		return RT_NULL;
	return rt_process_exec_env(RT_NULL,path,&cmd_line,0);
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
FINSH_FUNCTION_EXPORT_ALIAS(rt_process_exec_cmd, execp, exec process from a file);
#endif

#endif

/**
 * This function will destroy a process and release its resource.
 *
 * @param module the process to be destroyed.
 *
 * @return the operation status, RT_EOK on OK; -RT_ERROR on error
 */
rt_err_t rt_process_destroy(rt_process_t module)
{
    int i;
    struct rt_object *object;
    struct rt_list_node *list;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameter */
    RT_ASSERT(module != RT_NULL);
    RT_ASSERT(module->pid != 0);

    RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("rt_process_destroy: %8.*s\n",
                                   RT_NAME_MAX, module->parent.name));

    /* delete command line */
    if (module->module_cmd_line)
        rt_page_free(module->module_cmd_line,module->module_cmd_size/RT_MM_PAGE_SIZE);

    /* release process space memory */
    if (module->module_space)
        rt_page_free(module->module_space,module->module_size/RT_MM_PAGE_SIZE);

#ifdef RT_USING_SLAB
    if (module->page_cnt > 0)
    {
        for (i = 0; i < module->page_cnt; i ++)
        {
            if (((void **)module->page_array)[i])
                rt_page_free(((void **)module->page_array)[i],1);
        }
        rt_kprintf("Process: warning - memory leak 0x%08x-0x%08x %dk\n",
                module->vstart_addr+module->module_size,
                module->vstart_addr+module->module_size+module->page_cnt*RT_MM_PAGE_SIZE,
                module->page_cnt*4);
        module->page_cnt = 0;
    }
    if (module->page_array != RT_NULL)
        rt_free(module->page_array);
    if (module->page_mutex != RT_NULL)
        rt_mutex_delete(module->page_mutex);
    if (module->alarm != RT_NULL)
        rt_timer_delete(module->alarm);
#endif

    //关闭打开的文件
	for (i=0; i<DFS_FD_MAX; i++)
	{
		if (module->file_list[i] != 0)
		{
			int fileno = module->file_list[i]-1;
			struct dfs_fd *d = fd_get(fileno);
			if (d == NULL)
				continue;
			fd_put(d);
			if (d->ref_count >= 2)
			{
				fd_put(d);
				continue;
			}
			rt_kprintf("Process: warning - file leak %d/%d\n",i,fileno);
			close(fileno);
		}
	}

    /* switch tls */
    if (rt_pids_to == module->pid)
    {
        rt_pids_from = rt_pids_to = 0;
        mmu_switchtlb(0);
    }
    /* free tls */
    mmu_freetlb(module->pid);
    {register rt_ubase_t temp = rt_hw_interrupt_disable();
    free_pid(module->pid);
    release_tpid(module->tpid,module->exitcode);
    rt_hw_interrupt_enable(temp);}
    rt_process_kill(module,module->tpid,SIGCHLD);

    RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("rt_process_destroy: %8.*s finished\n",
                                   RT_NAME_MAX, module->parent.name));

    /* delete process object */
    rt_object_delete((rt_object_t)module);

    return RT_EOK;
}

/**
 * This function will unload a process from memory and release resources
 *
 * @param module the process to be unloaded
 *
 * @return the operation status, RT_EOK on OK; -RT_ERROR on error
 */
rt_err_t rt_process_unload(rt_process_t module, int exitcode)
{
    struct rt_object *object;
    struct rt_list_node *list;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameter */
    if (module == RT_NULL)
        return -RT_ERROR;

    /* vfork exit code */
    if (module->jmppid)
    {
    	//load thread used stack
    	rt_uint32_t jmp;
    	volatile char buf[256] = {0};
    	rt_memcpy((void *)module->jmpsp,module->jmpspbuf,module->jmpsplen);
    	//
    	jmp = module->tpid;module->tpid = module->jmppid;module->jmppid = 0;
    	longjmp(module->jmpbuf,jmp);
    }

    rt_enter_critical();
    /* delete all sub-threads */
    list = &module->thread_list;
    while (list->next != list)
    {
        object = rt_list_entry(list->next, struct rt_object, list);
        if (rt_object_is_systemobject(object) == RT_TRUE)
        {
            /* detach static object */
            rt_thread_detach((rt_thread_t)object);
        }
        else
        {
            /* delete dynamic object */
            rt_thread_delete((rt_thread_t)object);
        }
    }

    /* delete the alarm timer */
    if (module->alarm)
    {
        rt_timer_delete(module->alarm);
        module->alarm = RT_NULL;
    }

    /* delete the main thread of process */
    if (module->module_thread != RT_NULL)
    {
        module->exitcode = exitcode;
        rt_thread_delete(module->module_thread);
    }
    rt_exit_critical();

#ifdef RT_USING_HOOK
    if (rt_process_unload_hook != RT_NULL)
    {
        rt_process_unload_hook(module);
    }
#endif

    return RT_EOK;
}

/**
 * This function will find the specified process.
 *
 * @param name the name of process finding
 *
 * @return the process
 */
rt_process_t rt_process_find(const char *name)
{
    struct rt_object_information *information;
    struct rt_object *object;
    struct rt_list_node *node;

    extern struct rt_object_information rt_object_container[];

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* enter critical */
    rt_enter_critical();

    /* try to find device object */
    information = &rt_object_container[RT_Object_Class_Process];
    for (node = information->object_list.next;
         node != &(information->object_list);
         node = node->next)
    {
        object = rt_list_entry(node, struct rt_object, list);
        if (rt_strncmp(object->name, name, RT_NAME_MAX) == 0)
        {
            /* leave critical */
            rt_exit_critical();

            return (rt_process_t)object;
        }
    }

    /* leave critical */
    rt_exit_critical();

    /* not found */
    return RT_NULL;
}
RTM_EXPORT(rt_process_find);

rt_uint32_t rt_process_brk(rt_process_t module, rt_uint32_t addr)
{
    rt_uint32_t base = module->vstart_addr+module->module_size;
    if(addr <= base + module->page_cnt*RT_MM_PAGE_SIZE)
    {
        int idx = (addr<=base)?(0):(RT_ALIGN(addr-base,RT_MM_PAGE_SIZE)/RT_MM_PAGE_SIZE);
        int count = module->page_cnt-idx;
        if (module->page_cnt > 0)
            mmu_userunmap(module->pid,base+idx*RT_MM_PAGE_SIZE,
                    count*RT_MM_PAGE_SIZE,1);
        while (count --)
        {
            rt_page_free(((void **)module->page_array)[idx ++],1);
            module->page_cnt --;
        }
    }
    else
    {
        //limit memory
        int npage = addr-(base+module->page_cnt*RT_MM_PAGE_SIZE);
        npage = RT_ALIGN(npage,RT_MM_PAGE_SIZE)/RT_MM_PAGE_SIZE;
        if (module->page_cnt >= PAGE_COUNT_MAX)
            return base + module->page_cnt*RT_MM_PAGE_SIZE;
        else if (module->page_cnt + npage > PAGE_COUNT_MAX)
            npage = PAGE_COUNT_MAX-module->page_cnt;
        //--limit memory
        void *ptr = rt_page_alloc(npage);
        if (ptr == RT_NULL)
            return base + module->page_cnt*RT_MM_PAGE_SIZE;
        mmu_usermap(module->pid,(rt_uint32_t)ptr,
                base+module->page_cnt*RT_MM_PAGE_SIZE,npage*RT_MM_PAGE_SIZE,1);
        while (npage --)
        {
            ((void **)module->page_array)[module->page_cnt] = ptr;
            module->page_cnt ++;
            ptr += RT_MM_PAGE_SIZE;
        }
    }
    return addr;
}

void *rt_process_conv_ptr(rt_process_t module, rt_uint32_t ptr, rt_int32_t size)
{
    rt_uint32_t inmem = module->vstart_addr+module->module_size+module->page_cnt*RT_MM_PAGE_SIZE;
    rt_uint32_t base = RT_ALIGN_DOWN(ptr,RT_MM_PAGE_SIZE);
    do
    {
        if (ptr == 0)
            break;
        if (base < module->vstart_addr - RT_PROCESS_MEMMAKE - RT_USING_PROCESS_STKSZ)
            break;
        if (base > inmem)
            break;
        //calc strlen
        if (size == 0)
        {
            char *buf = (char *)ptr;
            while (*buf)
            {
                if ((rt_uint32_t)buf > inmem)
                {
                    buf = RT_NULL;
                    break;
                }
                buf++;
                size++;
            }
            if (buf == RT_NULL)
                break;
            size++;
        }
        //calc char** buflen
        else if (size == -1)
        {
            char **buf = (char **)ptr;
            while (*buf)
            {
                if ((rt_uint32_t)buf > inmem)
                {
                    buf = RT_NULL;
                    break;
                }
                buf++;
                size+=sizeof(char *);
            }
            if (buf == RT_NULL)
                break;
            size+=sizeof(char *);
        }
        //err parm
        else if (size < -1)
        {
        	break;
        }
        //--calc real buflen
        rt_uint32_t end = RT_ALIGN(ptr+size,RT_MM_PAGE_SIZE);
        if (end > inmem)
            break;
        if(mmu_check_ptr(module->pid,base,end-base))
            return (void *)ptr;
    } while (0);

    rt_kprintf("\nthread - %.*s - ", RT_NAME_MAX, rt_thread_self()->name);
    rt_kprintf("data abort addr:%x size:%x\n", ptr, size);

	rt_process_unload(module,SIGSEGV);
	rt_schedule();
    return RT_NULL;
}

int rt_process_fork(rt_process_t module)
{
	int i,ret;
	rt_list_t list;
	if (module->jmppid || module->jmpsp || module->jmpsplen)
		return -ENOSYS;

    /* allocate process */
    rt_process_t forkmod = (struct rt_process *)rt_process_copy_object(RT_NULL,&module->parent);
    if (!forkmod)
        return -ENOMEM;

    {register rt_ubase_t temp = rt_hw_interrupt_disable();
    forkmod->pid = getempty_pid();
    if (forkmod->pid)
    {
        forkmod->tpid = getempty_tpid(forkmod->tpid);
        if (forkmod->tpid == 0)
        {
            free_pid(forkmod->pid);
            forkmod->pid = 0;
        }
    }
    rt_hw_interrupt_enable(temp);}
    if (forkmod->pid == 0)
    {
        rt_kprintf("Process: allocate pid failed.\n");
        rt_object_delete(&(forkmod->parent));

        return -ENOMEM;
    }

    /* allocate process space */
    forkmod->module_space = rt_page_alloc(forkmod->module_size/RT_MM_PAGE_SIZE);
    if (forkmod->module_space == RT_NULL)
    {
        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(forkmod->pid);
        free_tpid(forkmod->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate space failed.\n");
        rt_object_delete(&(forkmod->parent));

        return -ENOMEM;
    }

    /* zero all space */
    mmu_maketlb(forkmod->pid,0);
    mmu_usermap(forkmod->pid,(Elf32_Addr)forkmod->module_space,forkmod->vstart_addr,forkmod->module_size,0);

    /* init process object container */
    rt_process_init_object_container(forkmod);
    /* fix copy system object */

    forkmod->module_cmd_line = (rt_uint8_t*)rt_page_alloc(forkmod->module_cmd_size/RT_MM_PAGE_SIZE);
    if (!forkmod->module_cmd_line)
    {
        /* release process space memory */
        rt_page_free(forkmod->module_space,forkmod->module_size/RT_MM_PAGE_SIZE);

        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(forkmod->pid);
        free_tpid(forkmod->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate cmd buffer failed.\n");
        rt_object_delete(&(forkmod->parent));

        return -ENOMEM;
    }

    /* set process argument */
    mmu_usermap(forkmod->pid,(rt_uint32_t)forkmod->module_cmd_line,
            forkmod->vstart_addr-forkmod->module_cmd_size,forkmod->module_cmd_size,0);
    /* hold thread stack */
    mmu_userunmap(forkmod->pid,forkmod->vstart_addr-RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);
    mmu_userunmap(forkmod->pid,forkmod->vstart_addr-RT_USING_PROCESS_STKSZ-2*RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);
    mmu_userunmap(forkmod->pid,forkmod->vstart_addr-RT_USING_PROCESS_STKSZ-RT_ALIGN(sizeof(struct _reent),1024)-3*RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);

    /* create page array */
    forkmod->page_array = (void *)rt_malloc(PAGE_COUNT_MAX*sizeof(rt_uint32_t));
    forkmod->page_cnt = 0;
    /* initialize heap semaphore */
    forkmod->page_mutex = rt_mutex_create(module->page_mutex->parent.parent.name, RT_IPC_FLAG_FIFO);
    forkmod->alarm = (rt_timer_t)rt_process_copy_object(RT_NULL,(rt_object_t)module->alarm);

    if (!forkmod->page_array || !forkmod->page_mutex || !forkmod->alarm)
    {
        /* release process space memory */
        rt_page_free(forkmod->module_space,forkmod->module_size/RT_MM_PAGE_SIZE);

        /* delete command line */
        rt_page_free(forkmod->module_cmd_line,forkmod->module_cmd_size/RT_MM_PAGE_SIZE);

        if (forkmod->page_array != RT_NULL)
            rt_free(forkmod->page_array);
        if (forkmod->page_mutex != RT_NULL)
            rt_mutex_delete(forkmod->page_mutex);
        if (forkmod->alarm != RT_NULL)
            rt_timer_delete(forkmod->alarm);

        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(forkmod->pid);
        free_tpid(forkmod->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate mem manager failed.\n");
        rt_object_delete(&(forkmod->parent));

        return -ENOMEM;
    }

    /* create process thread */
    forkmod->module_thread = (struct rt_thread *)rt_process_copy_object(RT_NULL,(rt_object_t)module->module_thread);
    if (!forkmod->module_thread)
    {
        /* release process space memory */
        rt_page_free(forkmod->module_space,forkmod->module_size/RT_MM_PAGE_SIZE);

        /* delete command line */
        rt_page_free(forkmod->module_cmd_line,forkmod->module_cmd_size/RT_MM_PAGE_SIZE);

        if (forkmod->page_array != RT_NULL)
            rt_free(forkmod->page_array);
        if (forkmod->page_mutex != RT_NULL)
            rt_mutex_delete(forkmod->page_mutex);
        if (forkmod->alarm != RT_NULL)
            rt_timer_delete(forkmod->alarm);

        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        free_pid(forkmod->pid);
        free_tpid(forkmod->tpid);
        rt_hw_interrupt_enable(temp);}

        rt_kprintf("Process: allocate thread failed.\n");
        rt_object_delete(&(forkmod->parent));

        return -ENOMEM;
    }
    /* set process process_id */
    forkmod->module_thread->process_id = (void *)forkmod;

    /* copy mempage info */
    for (i = 0; i < module->page_cnt; i ++)
    {
        ((void **)forkmod->page_array)[i] = rt_page_alloc(1);
        if (!((void **)forkmod->page_array)[i])
        {
            /* release process space memory */
            rt_page_free(forkmod->module_space,forkmod->module_size/RT_MM_PAGE_SIZE);

            /* delete command line */
            rt_page_free(forkmod->module_cmd_line,forkmod->module_cmd_size/RT_MM_PAGE_SIZE);

            for (i = 0; i < forkmod->page_cnt; i ++)
                rt_page_free(((void **)forkmod->page_array)[i],1);
            forkmod->page_cnt = 0;
            if (forkmod->page_array != RT_NULL)
                rt_free(forkmod->page_array);
            if (forkmod->page_mutex != RT_NULL)
                rt_mutex_delete(forkmod->page_mutex);
            if (forkmod->alarm != RT_NULL)
                rt_timer_delete(forkmod->alarm);

            {register rt_ubase_t temp = rt_hw_interrupt_disable();
            free_pid(forkmod->pid);
            free_tpid(forkmod->tpid);
            rt_hw_interrupt_enable(temp);}

            rt_kprintf("Process: allocate failed.\n");
            rt_thread_delete(forkmod->module_thread);
            rt_object_delete(&(forkmod->parent));

            return -ENOMEM;
        }
        mmu_usermap(forkmod->pid,(rt_uint32_t)(((void **)forkmod->page_array)[i]),
                forkmod->vstart_addr+forkmod->module_size+forkmod->page_cnt*RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE,0);
        forkmod->page_cnt++;
    }

    /* dump open file */
	for (i=0; i<DFS_FD_MAX; i++)
	{
		if (module->file_list[i] != 0)
		{
			int fileno = module->file_list[i]-1;
			forkmod->file_list[i] = module->file_list[i];
			fd_get(fileno);
		}
	}

    /* dump thread stack */
    extern int rt_hw_context_save(rt_uint32_t ret, rt_uint32_t to);
    ret = rt_hw_context_save(0, (rt_uint32_t)&forkmod->module_thread->sp);
    if (ret != 0)
    {
        rt_memcpy(forkmod->module_space, (void *)forkmod->vstart_addr, forkmod->module_size);
        rt_memcpy(forkmod->module_cmd_line, (void *)forkmod->vstart_addr-forkmod->module_cmd_size, forkmod->module_cmd_size);
        for (i = 0; i < forkmod->page_cnt; i ++)
        {
            rt_memcpy(((void **)forkmod->page_array)[i],
                    (void *)forkmod->vstart_addr+forkmod->module_size+i*RT_MM_PAGE_SIZE,RT_MM_PAGE_SIZE);
        }
        ret = forkmod->tpid;
        __asm volatile("add sp, #16*4");
        //开放调度
        {register rt_ubase_t temp = rt_hw_interrupt_disable();
        if (!rt_list_isempty(&(module->module_thread->tlist)))
            rt_list_insert_before(&(module->module_thread->tlist), &(forkmod->module_thread->tlist));
        rt_hw_interrupt_enable(temp);}
    }
    return ret;
}

int rt_process_vfork(rt_process_t module)
{
	//save thread used stack
	rt_thread_t tid = rt_thread_self();
	__asm volatile("mov %0,sp":"=r" (module->jmpsp));
	module->jmpsplen = module->jmpsp-tid->stack_addr-tid->stack_size;
	if (module->jmpsplen > sizeof(module->jmpspbuf)) module->jmpsplen = sizeof(module->jmpspbuf);
	rt_memcpy(module->jmpspbuf,module->jmpsp,module->jmpsplen);
	//
    int jmp = setjmp(module->jmpbuf);
    if (jmp != 0)
    {
    	if (module->jmpsp == 0 && module->jmpsplen)
        	return -ENOMEM;
    	module->jmpsp = 0;
    	module->jmpsplen = 0;
    	return jmp;
    }
    {register rt_ubase_t temp = rt_hw_interrupt_disable();
    module->jmppid = module->tpid;
    module->tpid = getempty_tpid(module->jmppid);
    rt_hw_interrupt_enable(temp);}
    if (module->tpid == 0)
    {
    	module->tpid = module->jmppid;
    	module->jmpsp = 0;
    	module->jmpsplen = 0;
    	return -ENOMEM;
    }
    return 0;
}

int rt_process_execve(rt_process_t module, const char*file, const char **argv, const char **envp)
{
	struct stat st;
	if (lstat(file,&st) != 0)
		return -ENOENT;

	const char *realfile = file;
	char filename[200];
	if (S_ISLNK(st.st_mode))
	{
		int ret = readlink(file,filename,sizeof(filename));
		if (ret < 0)
			return ret;
		filename[ret] = 0;
		realfile = filename;
	}

	rt_process_t process = rt_process_exec_env(module,realfile,argv,envp);
	if (process != RT_NULL);
	{
		if (!module->jmppid && !module->jmpsp && !module->jmpsplen)
			module->tpid = 0;
		rt_process_unload(module,SIGSTOP);
		rt_schedule();
	}
    return -1;
}

int rt_process_waitpid(rt_process_t module, pid_t pid, int* status, int opt)
{
	int i;
	int waitid = 0,wait = ((opt&WNOHANG) != WNOHANG);
	int find;
	int result;

	//需要等待时分配等待事件
	if (wait)
	{
		register rt_ubase_t temp = rt_hw_interrupt_disable();
		waitid = __rt_ffs(mod_waitp);
		RT_ASSERT(waitid != 0);
		mod_waitp &= ~(1<<(waitid-1));
		rt_event_recv(&mod_eventp,(1<<(waitid-1)),RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,0);
		rt_hw_interrupt_enable(temp);
	}

	do {
	find = 0;
	result = -1;
	rt_enter_critical();
	for(i=0; i<MAX_PID_SIZE; i++)
	{
		if (pidinfo[i][0] < 100)
			continue;
		//判断是否有子进程的存在
		if (pid < -1)
		{
			if (pidinfo[i][2] != -pid)
				continue;
		}
		else if (pid == -1)
		{
			if (pidinfo[i][1] != module->tpid)
				continue;
		}
		else if (pid == 0)
		{
			if (pidinfo[i][2] != pidinfo[module->tpid-1][2])
				continue;
		}
		else
		{
			if (i+1 != pid)
				continue;
		}
		find = 1;
		//判断进程是否退出
		if (pidinfo[i][0] == 102)
		{
			if (status)
				*status = pidbuf[i];
			//搜索进程是否还有被占用的存在
			pidinfo[i][0] = 1;
			if (!find_tpid(i+1,i+1))
				free_tpid(i+1);
			if (pidinfo[i][2] != 0
					&& pidinfo[pidinfo[i][2]-1][0] < 100 && pidinfo[pidinfo[i][2]-1][0] > 0
					&& !find_tpid(pidinfo[i][2],i+1))
				free_tpid(pidinfo[i][2]);
			if (pidinfo[i][3] != 0
					&& pidinfo[pidinfo[i][3]-1][0] < 100 && pidinfo[pidinfo[i][2]-1][0] > 0
					&& !find_tpid(pidinfo[i][3],i+1))
				free_tpid(pidinfo[i][3]);
			result = i+1;
			break;
		}
	}
	rt_exit_critical();

	//没有子进程或者已经找到子进程
	if (find == 0 || result != -1)
	{
		if (find == 0)
			result = -ECHILD;
		break;
	}

	//是否需要进行等待
	if (!wait)
		break;
	rt_event_recv(&mod_eventp,(1<<(waitid-1)),RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,30000,0);
	}while (1);

	//释放已经分配的事件
	if (wait)
	{
		register rt_ubase_t temp = rt_hw_interrupt_disable();
		mod_waitp |= (1<<(waitid-1));
		rt_hw_interrupt_enable(temp);
	}
	if (result > 0)
	{
        RT_DEBUG_LOG(RT_DEBUG_PROCESS, ("waitpid: %8.*s wait chile:%d\n",
                                       RT_NAME_MAX, module->parent.name,result));
	}
	return result;
}

int rt_process_savefile(rt_process_t module, int fileno)
{
	int i=0;
	for (i=0; i<DFS_FD_MAX; i++)
	{
		if (module->file_list[i] == 0)
		{
			module->file_list[i] = fileno+1;
			return i;
		}
	}
	return -1;
}

int rt_process_convfile(rt_process_t module, int fileno)
{
	if (fileno >= 0 && fileno< DFS_FD_MAX)
		return module->file_list[fileno]-1;
	return -1;
}

int rt_process_setfile(rt_process_t module, int fileno, int newfileno)
{
	if (fileno >= 0 && fileno< DFS_FD_MAX)
	{
		module->file_list[fileno] = newfileno+1;
		return newfileno;
	}
	return -1;
}

int rt_process_clearfile(rt_process_t module, int fileno)
{
	if (fileno >= 0 && fileno< DFS_FD_MAX)
	{
		module->file_list[fileno] = 0;
		return 0;
	}
	return -1;
}

void rt_process_wait(int delay)
{
    if (rt_event_recv(&mod_eventp,1,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,delay,0) != RT_EOK)
        return;
    free_tpid(1);
#ifdef RT_USING_FINSH
    tty_rx_inxpz = 1;
    rt_kprintf(FINSH_PROMPT);
#endif
}

int rt_process_kill(rt_process_t module, int pid, int sig)
{
    struct rt_object_information *information;
    struct rt_process *find = RT_NULL;
    struct rt_list_node *node;

    /* enter critical */
    rt_enter_critical();

    /* try to find device object */
    extern struct rt_object_information rt_object_container[];
    information = &rt_object_container[RT_Object_Class_Process];
    for (node = information->object_list.next;
         node != &(information->object_list);
         node = node->next)
    {
        struct rt_process *object = (struct rt_process *)rt_list_entry(node, struct rt_object, list);
        if (pid >0 &&
        		((sig != SIGCHLD && object->tpid == pid)
        				|| (sig ==SIGCHLD && object->tpid == pidinfo[pid-1][1])))
        {
            //找到子进程或者子进程的父进程
            find = object;
            break;
        }
    }

    /* leave critical */
    rt_exit_critical();

    rt_kprintf("kill %d %d\n",pid,sig);
    if (find == RT_NULL)
        return -ESRCH;

    //处理退出进程
    if (sig == SIGKILL)
    {
		rt_process_unload(find,SIGKILL);
		rt_schedule();
		return 0;
    }
    //特殊处理子进程退出事件
    if (sig == SIGCHLD)
    {
    	if ((find->sigact[sig].sa_handler == SIG_IGN)
    			|| (find->sigact[sig].sa_handler == SIG_ERR)
    			|| (find->sigact[sig].sa_flags & SA_NOCLDSTOP))
    	{
    		//忽略信号，不产生僵尸进程
    		rt_process_waitpid(module,pid,0,WNOHANG);
    		return 0;
    	}
    }
    //信号被忽略，不用进行处理
	if ((find->sigact[sig].sa_handler == SIG_IGN)
			|| (find->sigact[sig].sa_handler == SIG_ERR))
		return 0;

	int dosched = 0;
	register rt_ubase_t temp = rt_hw_interrupt_disable();
	find->siginfo |= (1<<sig);

	do
	{
		//信号被阻止不用立刻处理
		if (find->sigset & (1<<sig))
			break;
		//程序正在运行不用唤醒
		if (find->module_thread->stat != RT_THREAD_SUSPEND)
			break;
		//不带超时时间的休眠，不能唤醒，因为无法恢复到原来的状态
		if (!(find->module_thread->thread_timer.parent.flag & RT_TIMER_FLAG_ACTIVATED))
			break;
		//唤醒并设置错误
		find->module_thread->error = -RT_EINTR;
		rt_thread_resume(find->module_thread);
		dosched = 1;
	} while (0);
	rt_hw_interrupt_enable(temp);

	//强制调度一下
	if (dosched)
		rt_schedule();
	return 0;
}
#endif
