DIALECT = -std=c11
CFLAGS += $(DIALECT) -O3 -g -W -Wall
LIBS = -lm

CFLAGS += $(shell pkg-config --cflags libbladeRF)
CFLAGS += $(shell pkg-config --cflags libhackrf)
CFLAGS += $(shell pkg-config --cflags libiio libad9361)

.PHONY: all bladeplayer hackplayer limeplayer plutoplayer clean
all: bladeplayer hackplayer limeplayer plutoplayer
	
%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

bladeplayer: bladeplayer.o $(SDR_OBJ) $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(shell pkg-config --libs libbladeRF)

hackplayer: hackplayer.o $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(shell pkg-config --libs libhackrf)

limeplayer: limeplayer.c
	gcc -O2 -Wall -o limeplayer limeplayer.c -lLimeSuite

plutoplayer: plutoplayer.o $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(shell pkg-config --libs libiio libad9361)

clean:
	rm -f *.o  bladeplayer hackplayer limeplayer plutoplayer
