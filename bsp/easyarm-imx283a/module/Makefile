CC = /opt/bin/arm-none-eabi-gcc.exe
CFLAGS = -mcpu=arm926ej-s -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-variable -Os
LIBS = -Wl,--gc-sections,-n,--section-start=.init=0x10000000 --specs=linux.specs
OBJS = main.o

test:$(OBJS)
	$(CC) $(CFLAGS) -o test $(OBJS) $(LIBS)

release:test
	$(STRIP) test

clean:
	rm -f *.o
	rm -f test
