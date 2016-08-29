CC = ../../bin/arm-none-eabi-gcc
AR = ../../bin/arm-none-eabi-ar
CFLAGS = -mcpu=arm926ej-s -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-variable -Os
CFLAGS += -isystem../include
LIBS = -Wl,--gc-sections,-n,--section-start=.init=0x10000000 --specs=linux.specs
OBJS = main.o \
	syscalls.o \
	posix/regcomp.o \
	posix/regerror.o \
	posix/regexec.o \
	posix/regfree.o

%.o:%.c
	@echo CC $<
	@$(CC) $(CFLAGS) -c -o $@ $< 

rtt_syscall:$(OBJS)
	@echo AR rtt_syscall.a
	@$(AR) rc rtt_syscall.a $(OBJS)

rtt_do:$(OBJS)
	@echo LD rtt_do.mo
	@$(CC) -o rtt_do.mo $(CFLAGS) $(LIBS) $(OBJS)

clean:
	rm -f *.o
	rm -f posix/*.o
	rm -f rtt_syscall.a rtt_do.mo

