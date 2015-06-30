# Makefile for Linux etc.

.PHONY: all clean
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
	rm -f gpssim.o gpssim-lut.o gps-sdr-sim gps-sdr-sim-lut
