# Makefile for Linux etc.

.PHONY: all clean time
all: gps-sdr-sim

SHELL=/bin/bash
CC=gcc
CFLAGS=-O3 -Wall
LDFLAGS=-lm -lpthread

gps-sdr-sim: gpssim.o
	${CC} $< ${LDFLAGS} -o $@

clean:
	rm -f gpssim.o gps-sdr-sim *.bin

time: gps-sdr-sim
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 1
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 8
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16
