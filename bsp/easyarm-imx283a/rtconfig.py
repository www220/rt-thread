import os
ARCH     = 'arm'
CPU      = 'arm926'
CROSS_TOOL = 'gcc'

#ARCH     = 'sim'
#CPU      = 'win32'
#CROSS_TOOL = 'msvc'

#------- toolchains path -------------------------------------------------------
if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')
    if  CROSS_TOOL == 'msvc':
        ARCH = 'sim'
        CPU  = 'win32'
    else:
        ARCH = 'arm'
        CPU  = 'arm926'
        CROSS_TOOL = 'gcc'
    print 'CROSS_TOOL = ' + CROSS_TOOL

if  CROSS_TOOL == 'gcc':
	PLATFORM 	= 'gcc'
	EXEC_PATH 	= r'C:/Program Files (x86)/GNU Tools ARM Embedded/4.9 2015q2/bin'
elif CROSS_TOOL == 'msvc':
	PLATFORM 	= 'cl'
	EXEC_PATH 	= r''

if os.getenv('RTT_EXEC_PATH'):
	EXEC_PATH = os.getenv('RTT_EXEC_PATH')

#BUILD = 'debug'
BUILD = 'release'

CORE = 'arm926ej-s'
MAP_FILE = 'rtthread_imx283.map'
TARGET_NAME = 'rtthread.bin'

#------- GCC settings ----------------------------------------------------------
if PLATFORM == 'gcc':
    PREFIX = 'arm-none-eabi-'
    CC = PREFIX + 'gcc'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'axf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'

    DEVICE = ' -mcpu=arm926ej-s -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-variable'
    CFLAGS = DEVICE
    AFLAGS = '-c'+ DEVICE + ' -x assembler-with-cpp'
    AFLAGS += ' -Iplatform'
    LFLAGS = DEVICE
    LFLAGS += ' -Wl,--gc-sections,-cref,-Map=' + MAP_FILE
    LFLAGS += ' -T imx283_ram.ld'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -Os'

    POST_ACTION = OBJCPY + ' -O binary $TARGET ' + TARGET_NAME + '\n' 
    POST_ACTION += SIZE + ' $TARGET\n'

    M_CFLAGS = CFLAGS + ' -mlong-calls -fPIC '
    M_LFLAGS = DEVICE + ' -Wl,--gc-sections,-z,max-page-size=0x4 -shared -fPIC -nostdlib'
    M_LFLAGS += ' -T imx283_dll.ld'

elif PLATFORM == 'cl':
    PREFIX = ''
    TARGET_EXT = 'exe'
    AS = PREFIX + 'cl'
    CC = PREFIX + 'cl'
    AR = PREFIX + 'cl'
    LINK = PREFIX + 'link'
    AFLAGS = ''

    CFLAGS = ' /ZI /nologo /W3 /WX- /MTd /Od /Oy- /Gm /EHsc /RTC1 /GS /fp:precise '
    CFLAGS += ' /Zc:wchar_t /Zc:forScope /Gd /analyze- /errorReport:queue '
    LFLAGS = ' /NOLOGO /NODEFAULTLIB:LIBCMTD.lib /SUBSYSTEM:CONSOLE '
    LFLAGS += ' /DYNAMICBASE /NXCOMPAT /MACHINE:X86 /ERRORREPORT:QUEUE /DEBUG /FORCE:MULTIPLE '

    CPATH = ''
    LPATH = ''

    POST_ACTION = ''

    M_CFLAGS = CFLAGS
    M_LFLAGS = LFLAGS