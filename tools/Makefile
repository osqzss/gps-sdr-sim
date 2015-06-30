# Makefile for Linux etc.

.PHONY: all clean
all: nmea2um

SHELL=/bin/bash
CC=gcc
CFLAGS=-O3
LDFLAGS=-lm

nmea2um: nmea2um.o
	${CC} $< ${LDFLAGS} -o $@

clean:
	rm -f nmea2um.o nmea2um
