all: tilt1 spectrum spectrum2 nearest

CC=g++
CFLAGS=-Wall -g `pkg-config opencv --cflags --libs`
LDFLAGS=-lao -lusb -lfreenect -I /usr/include/libusb-1.0/ -I libfreenect/include

SOURCES=tilt1.cpp spectrum.cpp spectrum2.cpp nearest.cpp
EXECUTABLES=$(SOURCES:.cpp=)

clean:
	rm $(EXECUTABLES)

tilt1: tilt1.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

spectrum: spectrum.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

spectrum2: spectrum2.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

nearest: nearest.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<
