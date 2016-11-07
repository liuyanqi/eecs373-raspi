CC = g++

#CFLAGS = -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -fPIC `pkg-config --cflags glib-2.0`
CFLAGS = -g `pkg-config --cflags glib-2.0`
INCLUDES = -I/usr/local/include  -I/usr/include/glib-2.0
LDFLAGS = -L/usr/local/lib  -L../lib 
LIBS =`pkg-config --libs glib-2.0` -lgattlib

CSRCS = $(shell ls *.c)
COBJS = $(CSRCS:%.c=%.o)
TARGET = ./main


%.o: %.c %.h
	@$(CC) $(INCLUDES)$(CFLAGS)  -o $@ -c $<

all: $(TARGET)
	@/bin/true

#$(TARGET): 
#	$(GXX) -o $@ camera.cpp serial.c serial.h $(LDFLAGS) $(LIBS)	

$(TARGET): $(COBJS) $(COBJS)
	$(CC) -o $@ $^ $(LDFLAGS) $(LIBS)

clean:
	@rm -rf *.o $(TARGET)
