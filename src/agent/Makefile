# Makefile

all: default

CC=gcc

LD = gcc -o
LDFLAGS = -lstdc++

SRCS = client.cpp server.cpp

OBJS = $(SRCS:%.c=%.o)

.PHONY: all default clean cli ser

default: cli ser

cli: client.o
	$(LD) client client.o $(LDFLAGS)

ser: server.o
	$(LD) server server.o $(LDFLAGS)

clean:
	rm -f $(OBJS) *.a *.lib *.exp *.pdb x264 x264.exe .depend TAGS


