CC = gcc
CXX = g++

CPPFLAGS = -I..
CFLAGS = -g -std=gnu99  -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -g  -O4
LDFLAGS = -lpthread -lm -luvc -ljpeg

TARGETS := apriltag_demo opencv_demo

.PHONY: all
all: apriltag_demo

apriltag_demo: apriltag_demo.o ../libapriltag.a libuvc.h
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

opencv_demo: opencv_demo.o ../libapriltag.a
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
