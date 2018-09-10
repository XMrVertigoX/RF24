AR  := $(CROSS_COMPILE)ar
CC  := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
RM  := rm -rf

SOURCE_FILES := $(shell find -type f -name *.cpp)
OBJECT_FILES := $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))
DEPENDENCIES := $(patsubst %.cpp,%.d,$(wildcard src/*.cpp))
OUTPUT_FILES := libnrf24l01.a

CPPFLAGS += -Iinc
CPPFLAGS += -MD

# -----

.PHONY: all clean

all: $(OUTPUT_FILES)

clean:
	$(RM) $(OUTPUT_FILES) $(OBJECT_FILES) $(DEPENDENCIES)

# -----

libnrf24l01.a: $(OBJECT_FILES)
	@$(AR) r $@ $^

# -----

-include $(DEPENDENCIES)
