# Makefile for Linux etc.

.PHONY: all clean test
all: gps-sdr-sim

SHELL=/bin/bash
CC=gcc
CFLAGS=-fopenmp -O3
LDFLAGS=-lm -fopenmp

gps-sdr-sim: gpssim.o
	${CC} $< ${LDFLAGS} -o $@

clean:
	rm -f gpssim.o gps-sdr-sim *.bin

test: gps-sdr-sim
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 8
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "1a1aa6de6ee7faf58c1544a62aabc900"
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "5b33200201e8e90e6f29a5236dd89a2d"
