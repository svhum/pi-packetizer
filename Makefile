# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := $(HOME)/.local
endif

GCC=gcc
GPP=g++
CFLAGS = -O3 -D_GNU_SOURCE
CPPFLAGS = -fpermissive

packetizer: packetizer.c
	$(GCC) -o $@ $^ -lasound

freqmeas: freqmeas.cpp si5351.h si5351.cpp
	$(GPP) $(CPPFLAGS) -o $@ $^ -lasound

si5351test: si5351test.cpp si5351.h si5351.cpp
	$(GPP) $(CPPFLAGS) -o $@ $^

sdr-receiver-hpsdr: sdr-receiver-hpsdr.c
	$(GCC) $(CFLAGS) -o $@ $^ -lm -lpthread -lasound

spitest: spitest.c
	$(GCC) -o $@ $^

serial-test: serial-test.c
	$(GCC) -o $@ $^

#checkpointers: checkpointers.c
#	$(GCC) -o $@ $^

reset-cmod: reset-cmod.c
	$(GCC) -o $@ $^ -lwiringPi

install: reset-cmod sdr-receiver-hpsdr
	cp reset-cmod $(PREFIX)/bin
	cp sdr-receiver-hpsdr $(PREFIX)/bin

clean:
	rm -f packetizer sdr-receiver-hpsdr reset-cmod spitest freqmeas
