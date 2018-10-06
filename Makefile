AR   := $(CROSS_COMPILE)ar
CC   := $(CROSS_COMPILE)gcc
CXX  := $(CROSS_COMPILE)g++
RM   := rm -rf

INCLUDE_DIRS := inc tests/inc
SOURCE_FILES := $(wildcard src/*.cpp) $(wildcard tests/src/*.cpp)
OBJECT_FILES := $(patsubst %.cpp,%.o,$(SOURCE_FILES))
DEPENDENCIES := $(patsubst %.cpp,%.d,$(SOURCE_FILES))

CFLAGS += -std=c17

CXXFLAGS += -std=c++17

CPPFLAGS += $(addprefix -I,$(INCLUDE_DIRS)) -MD -MP

# -----

.PHONY: all clean tests

all: catch
	./catch -s

clean:
	$(RM) $(OBJECT_FILES) $(DEPENDENCIES)

catch: tests/catch_main.o $(OBJECT_FILES)
	$(CXX) $^ -o $@

# -----

-include $(DEPENDENCIES)
