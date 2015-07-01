# Makefile for Linux etc.

.PHONY: all clean test test-lut test-nolut
all: gps-sdr-sim gps-sdr-sim-lut

SHELL=/bin/bash
CC=gcc
CFLAGS=-fopenmp -O3
LDFLAGS=-lm -fopenmp

gps-sdr-sim: gpssim.o
	${CC} $< ${LDFLAGS} -o $@

gps-sdr-sim-lut: gpssim-lut.o
	${CC} $< ${LDFLAGS} -o $@

gpssim-lut.o: gpssim.c
	${CC} -c -D_SINE_LUT ${CFLAGS} $< -o $@

clean:
	rm -f gpssim.o gpssim-lut.o gps-sdr-sim gps-sdr-sim-lut *.bin

test: test-lut
test-lut: gps-sdr-sim-lut
	time ./gps-sdr-sim-lut -e brdc3540.14n -u circle.csv -b 8
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "39a577af659440605c4ebbe178f4c4e3"
	time ./gps-sdr-sim-lut -e brdc3540.14n -u circle.csv -b 16
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "bdd460893ad73b19412fc1757e62ccf9"

test-nolut: gps-sdr-sim
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 8
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "f4beb0857f82038d0465eb9934009edd"
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16
	test "$$(md5sum gpssim.bin | awk '{print $$1}')" == "10403720cb3483515f470fdea09e02ed"
