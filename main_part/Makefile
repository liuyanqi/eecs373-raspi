CC = gcc
GXX = g++

CFLAGS = -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -pthread -fPIC
CFLAGS2 =  -g `pkg-config --cflags glib-2.0`
GXXFLAGS =  -Wall -g `pkg-config --cflags opencv``pkg-config --cflags glib-2.0`
INCLUDES = -I/usr/local/include  -I../include -I/usr/include/glib-2.0
LDFLAGS = -L/usr/local/lib  -L../lib
LIBS = `pkg-config --libs opencv` `pkg-config --libs glib-2.0` -lgattlib -lraspicam -lraspicam_cv -lwiringPi -lpthread

CSRCS = $(shell ls *.c)
CXXSRCS = $(shell ls *.cpp)
CXXOBJS = $(CXXSRCS:%.cpp=%.o)
COBJS = $(CSRCS:%.c=%.o)
TARGET = ./bin/main

%.o: %.cpp
	@$(GXX) $(GXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.c %.h
	@$(GXX) $(CFLAGS2) -o $@ -c $<

all: $(TARGET)
	@/bin/true

#$(TARGET): 
#	$(GXX) -o $@ camera.cpp serial.c serial.h $(LDFLAGS) $(LIBS)	

$(TARGET): $(COBJS) $(CXXOBJS)
	$(GXX) -o $@ $^ $(LDFLAGS) $(LIBS)

clean:
	@rm -rf *.o $(TARGET)
