CC=gcc
CFLAGS=-g -O0 -std=gnu99 -lpthread
DEPS = crc16.h main.h xlgyro_data_processor.h
OBJ = crc16.o main.o xlgyro_data_processor.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

xlgyrod: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o