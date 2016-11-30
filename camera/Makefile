CC = gcc
GXX = g++

CFLAGS = -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -pthread -fPIC -I.
GXXFLAGS = -Wall -g `pkg-config --cflags opencv`
INCLUDES = -I/usr/local/include  -I../include
LDFLAGS = -L/usr/local/lib  -L../lib
LIBS = `pkg-config --libs opencv` -lraspicam -lraspicam_cv -lwiringPi -lpthread

CSRCS = $(shell ls *.c)
CXXSRCS = $(shell ls *.cpp)
CXXOBJS = $(CXXSRCS:%.cpp=%.o)
COBJS = $(CSRCS:%.c=%.o)
TARGET = ./bin/main

%.o: %.cpp
	@$(GXX) $(GXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.c %.h
	@$(GXX) $(CXXFLAGS) -o $@ -c $<

all: $(TARGET)
	@/bin/true

#$(TARGET): 
#	$(GXX) -o $@ camera.cpp serial.c serial.h $(LDFLAGS) $(LIBS)	

$(TARGET): $(COBJS) $(CXXOBJS)
	$(GXX) -o $@ $^ $(LDFLAGS) $(LIBS)

clean:
	@rm -rf *.o $(TARGET)
