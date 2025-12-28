CFLAGS += -Wall -Wextra

all:
	$(CC) $(CFLAGS) crc8-ccitt.c main.c
