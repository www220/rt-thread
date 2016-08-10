CC = ../../bin/arm-none-eabi-gcc
CFLAGS = -mcpu=arm926ej-s -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-variable -Os
CFLAGS += -isystem../include
LIBS = -Wl,--gc-sections,-n,--section-start=.init=0x10000000 --specs=linux.specs
OBJS = main.o \
	syscalls.o \
	regcomp.o \
	regerror.o \
	regexec.o \
	regfree.o

%.o:%.c
	@$(CC) $(CFLAGS) -c -o $@ $< 
	@echo CC $<

test:$(OBJS)
	@$(CC) -o test $(CFLAGS) $(LIBS) $(OBJS)
	@echo LD test

release:test
	$(STRIP) test

clean:
	rm -f *.o
	rm -f test
