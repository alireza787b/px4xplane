# Detect the operating system
UNAME_S := $(shell uname -s)

# Compiler settings
ifeq ($(UNAME_S),Linux)
    CXX = g++
    LIBS = -lXPLM -lXPWidgets -lGL -lglut
    TARGET = linux.xpl
    CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -O3 -Wno-extra-qualification -DLIN=1 -DXPLM200 -DXPLM300 -DXPLM301
else
    CXX = clang++
    LIBS = -framework XPLM -framework XPWidgets
    TARGET = mac.xpl
    CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -O3 -Wno-extra-qualification -DAPL=1 -DIBM=1 -DXPLM200 -DXPLM300 -DXPLM301
    FRAMEWORK_DIRS = -F./lib/SDK/Libraries/Mac
endif

# Include and library paths
INCLUDE_DIRS = -I/usr/include -I./include -I./lib/SDK/CHeaders/XPLM -I./lib/mavlink/c_library_v2/all -I./lib/simpleini

LIBRARY_DIRS = -L./lib 

# Source files
SRCS = src/ConfigManager.cpp \
       src/configReader.cpp \
       src/ConnectionManager.cpp \
       src/DataRefManager.cpp \
       src/MAVLinkManager.cpp \
       src/px4xplane.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Rules
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -shared -o $@ $^ $(LIBRARY_DIRS) $(LIBS) $(FRAMEWORK_DIRS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE_DIRS)

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
