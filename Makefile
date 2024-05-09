GCC=gcc
CFLAGS = -O3 -D_GNU_SOURCE

packetizer: packetizer.c
	$(GCC) -o $@ $^ -lasound

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

clean:
	rm -f packetizer sdr-receiver-hpsdr
