LIBS=-lm
CFLAGS=-D_GNU_SOURCE -ggdb -O3 $(DEFINES)
good_night: good_night.c
	cc -o $@ $(CFLAGS) $^ $(LIBS)
