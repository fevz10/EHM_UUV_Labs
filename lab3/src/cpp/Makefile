CC = g++
CFLAGS = -g -Wall -pthread
SRCS = lab3.cpp
PROG = lab3

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
