CFLAGS=-Ilmic
LDFLAGS=-lwiringPi

raspinode: raspinode-send-v1.cpp
	cd lmic && $(MAKE)
	$(CC) $(CFLAGS) -o raspinode raspinode-send-v1.cpp lmic/*.o $(LDFLAGS)

all: raspinode

.PHONY: clean

clean:
	rm -f *.o raspinode
