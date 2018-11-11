OBJS := src/nrf24.o src/nrf24_ll.o

CXXFLAGS += -std=gnu++17

.PHONY: all clean

all: libnrf24l01.a

clean:
	@$(RM) $(OBJS) libnrf24l01.a

libnrf24l01.a: $(OBJS)
	@$(AR) -q $@ $^
