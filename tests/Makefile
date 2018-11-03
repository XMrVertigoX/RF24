CC := g++

INCLUDE_DIRS := ./inc ../inc
SOURCE_FILES := $(wildcard ./*.cpp) $(wildcard ./src/*.cpp) $(wildcard ../src/*.cpp)
OBJECT_FILES := $(patsubst %.cpp,%.o,$(SOURCE_FILES))
DEPENDENCIES := $(patsubst %.cpp,%.d,$(SOURCE_FILES))

CFLAGS += -std=c17

CXXFLAGS += -std=c++17

CPPFLAGS += $(addprefix -I,$(INCLUDE_DIRS)) -MP -MD

# -----

all: main
	@./main

clean:
	@rm -f main $(OBJECT_FILES) $(DEPENDENCIES)

main: $(OBJECT_FILES)

# -----

-include $(DEPENDENCIES)
