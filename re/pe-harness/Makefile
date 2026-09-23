CFLAGS ?= -O1 -g -Wall -Wextra -Wno-unused-parameter
LDLIBS = -lusb-1.0

neatcap: neatcap.o pe_harness.o
	$(CC) -o $@ $^ $(LDLIBS)

%.o: %.c pe_harness.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f neatcap *.o
