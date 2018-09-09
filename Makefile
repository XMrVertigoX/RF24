CPPFLAGS += -Iinc
CPPFLAGS += -MD

SOURCE_FILES := $(wildcard src/*.cpp)
OBJECT_FILES := $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))
DEPENDENCIES := $(patsubst %.cpp,%.d,$(wildcard src/*.cpp))

all: libnrf24l01.a

clean:
	rm -rf $(OBJECT_FILES) $(DEPENDENCIES)

libnrf24l01.a: $(OBJECT_FILES)

-include $(DEPENDENCIES)
