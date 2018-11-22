CFLAGS=-I$(SYSROOT)/usr/include -O2 -Wall
LDFLAGS=-L$(SYSROOT)/usr/lib -lpthread -lrt
 
spitest-rt: spitest-rt.o spi.o gpio.o
clean:
-rm spitest-rt spitest-rt.o spi.o gpio.o
 
