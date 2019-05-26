CFLAGS=-Ilmic -g
LDFLAGS=-lwiringPi

raspinode: raspinode.cpp config.conf TinyGPS.cpp /usr/lib/libgps.a
	cd lmic && $(MAKE)
	$(CC) $(CFLAGS) -o raspinode raspinode.cpp lmic/*.o $(LDFLAGS) -lgps -lm

all: raspinode

.PHONY: clean

clean:
	cd lmic && $(MAKE) clean
	rm -f *.o raspinode
