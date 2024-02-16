GCC=gcc
CFLAGS = -O3 -D_GNU_SOURCE

packetizer: packetizer.c
	$(GCC) -o $@ $^ -lasound

sdr-receiver-hpsdr: sdr-receiver-hpsdr.c
	$(GCC) $(CFLAGS) -o $@ $^ -lm -lpthread -lasound

#checkpointers: checkpointers.c
#	$(GCC) -o $@ $^

clean:
	rm -f packetizer sdr-receiver-hpsdr
