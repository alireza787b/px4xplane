# Detect the operating system
UNAME_S := $(shell uname -s)

BUILDDIR := ./build
SRC_BASE := .
TARGET := linux.xpl

# Compiler settings
ifeq ($(UNAME_S),Linux)
    CXX = g++
    LIBS = -lGL -lGLU -lX11
    CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -O3 -fPIC -fvisibility=hidden -DLIN=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1
    INCLUDE_DIRS = -I$(SRC_BASE)/SDK/CHeaders/XPLM -I$(SRC_BASE)/SDK/CHeaders/Widgets
endif

# Source and object files
SRCS := $(wildcard $(SRC_BASE)/src/*.cpp)
OBJS := $(patsubst %,$(BUILDDIR)/%,$(notdir $(SRCS:.cpp=.o)))

# Rules
all: $(BUILDDIR) $(TARGET)

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -shared -o $(BUILDDIR)/$(TARGET) $^ $(LIBS)

$(BUILDDIR)/%.o: $(SRC_BASE)/src/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean

