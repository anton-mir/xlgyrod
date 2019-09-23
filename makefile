CC=gcc
CFLAGS=-g -O0 -std=gnu99
DEPS = crc16.h main.h
OBJ = crc16.o main.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

xlgyrod: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o