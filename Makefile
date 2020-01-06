# Makefile for Linux etc.

.PHONY: all clean time
all: gps-sdr-sim

SHELL=/bin/bash
CC=gcc
CFLAGS=-O3 -Wall -D_FILE_OFFSET_BITS=64
ifdef USER_MOTION_SIZE
CFLAGS+=-DUSER_MOTION_SIZE=$(USER_MOTION_SIZE)
endif
LDFLAGS=-lm

gps-sdr-sim: gpssim.o
	${CC} $< ${LDFLAGS} -o $@

gpssim.o: .user-motion-size gpssim.h

.user-motion-size: .FORCE
	@if [ -f .user-motion-size ]; then \
		if [ "`cat .user-motion-size`" != "$(USER_MOTION_SIZE)" ]; then \
			echo "Updating .user-motion-size"; \
			echo "$(USER_MOTION_SIZE)" >| .user-motion-size; \
		fi; \
	else \
		echo "$(USER_MOTION_SIZE)" > .user-motion-size; \
	fi;

clean:
	rm -f gpssim.o gps-sdr-sim *.bin .user-motion-size

time: gps-sdr-sim
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 1
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 8
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16

.FORCE:

YEAR?=$(shell date +"%Y")
Y=$(patsubst 20%,%,$(YEAR))
%.$(Y)n:
	wget -q ftp://cddis.gsfc.nasa.gov/gnss/data/daily/$(YEAR)/brdc/$@.Z -O $@.Z
	uncompress $@.Z
