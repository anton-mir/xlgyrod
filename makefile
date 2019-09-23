CC=gcc
CFLAGS=-g -O0 -std=gnu99 -lpthread -lrt
DEPS = crc16.h main.h xlgyroserver.h
OBJ = crc16.o main.o xlgyroserver.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

xlgyrod: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o