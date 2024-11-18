CC = gcc
CFLAGS = -Wall
SRCS = pc.c serial_linux.c
HEADERS = serial_linux.h ../avr_client_common/packets.h
OBJS = $(SRCS:.c=.o)
MAIN = pc
CROSS_PLATFROM = ../avr_client_common/serial_protocol.*

.PHONY: depend clean

all: $(MAIN)

$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) $(SRCS) $(HEADERS) $(CROSS_PLATFROM) -o $(MAIN)

*.o: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -c

clean:
	rm -rf *.h.gch
	rm -rf *.o
	rm -rf $(MAIN)
