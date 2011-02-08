all: spectrum gradients near_color

CC=g++
CFLAGS=-Wall -g `pkg-config opencv --cflags --libs`
LDFLAGS=-lao -lusb -lfreenect -I /usr/include/libusb-1.0/ -I libfreenect/include

SOURCES=spectrum.cpp gradients.cpp near_color.cpp
EXECUTABLES=$(SOURCES:.cpp=)

clean:
	rm $(EXECUTABLES)

spectrum: spectrum.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

gradients: gradients.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

near_color: near_color.cpp 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<
