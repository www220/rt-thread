CC = ../../bin/arm-none-eabi-gcc
AR = ../../bin/arm-none-eabi-ar
STRIP = ../../bin/arm-none-eabi-strip
CFLAGS = -mcpu=arm926ej-s -ffunction-sections -fdata-sections -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-variable -Os
CFLAGS += -isystem../include -I./posix -I./linux
LIBS = -Wl,--gc-sections,-n,-z,max-page-size=0x8,--section-start=.init=0x10000000 --specs=linux.specs
OBJS = main.o \
	syscalls.o \
	posix/collcmp.o \
	posix/collate.o \
	posix/regcomp.o \
	posix/regerror.o \
	posix/regexec.o \
	posix/regfree.o \
	posix/glob.o \
	posix/fnmatch.o \
	posix/execl.o \
	posix/execle.o \
	posix/execlp.o \
	posix/execv.o \
	posix/execve.o \
	posix/execvp.o \
	posix/popen.o \
	linux/sysconf.o \
	linux/strverscmp.o \
	linux/basename.o \
	linux/dirname.o \
	linux/realpath.o \
	linux/termios.o \
	linux/cfspeed.o \
	linux/tcdrain.o \
	linux/tcsendbrk.o \
	linux/getopt.o \
	linux/getopt1.o \
	libbb/pwd_grp.o

%.o:%.c
	@echo CC $<
	@$(CC) $(CFLAGS) -c -o $@ $< 

rtt_syscall:$(OBJS)
	@echo AR rtt_syscall.a
	@$(AR) -rc rtt_syscall.a $(OBJS)

rtt_do:$(OBJS)
	@echo LD rtt_do_un.mo
	@$(CC) -o rtt_do_un.mo $(CFLAGS) $(LIBS) $(OBJS)
	@echo STRIP rtt_do.mo
	@$(STRIP) -s -K _impure_ptr rtt_do_un.mo -o rtt_do.mo

clean:
	rm -f *.o
	rm -f posix/*.o
	rm -f linux/*.o
	rm -f libbb/*.o
	rm -f rtt_syscall.a rtt_do_un.mo rtt_do.mo

