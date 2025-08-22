#==============================================================================
# px4xplane (PixEagle) - Professional Cross-Platform Makefile
# X-Plane Plugin for PX4 SITL Integration
# Supports: macOS, Linux with debug/release configurations
#==============================================================================

# Project Information
PROJECT_NAME := px4xplane
VERSION := 2.5.0

# Automatic OS Detection
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# Determine if we're on macOS or Linux
ifeq ($(UNAME_S),Darwin)
    OS := macos
    PLATFORM_DEFINE := -DAPL=1
    SHARED_LIB_EXT := .xpl
    OUTPUT_NAME := mac.xpl
else ifeq ($(UNAME_S),Linux)
    OS := linux
    PLATFORM_DEFINE := -DLIN=1
    SHARED_LIB_EXT := .xpl
    OUTPUT_NAME := linux.xpl
else
    $(error Unsupported operating system: $(UNAME_S))
endif

# Build Configuration (default: release)
CONFIG ?= release
BUILD_DIR := build/$(OS)/$(CONFIG)
OUTPUT_DIR := $(BUILD_DIR)

# Compiler Configuration
CXX := g++
ifeq ($(OS),macos)
    CXX := clang++
endif

# Standard Compiler Flags
CXXFLAGS_COMMON := -std=c++17 -fPIC -Wall -Wextra -pedantic -Wno-extra-qualification
CXXFLAGS_COMMON += $(PLATFORM_DEFINE) -DXPLM200=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1 -DXPLM303=1

# Configuration-specific flags
ifeq ($(CONFIG),debug)
    CXXFLAGS_CONFIG := -g -O0 -DDEBUG -D_DEBUG
    $(info Building in DEBUG mode...)
else ifeq ($(CONFIG),release)
    CXXFLAGS_CONFIG := -O3 -DNDEBUG -DRELEASE
    $(info Building in RELEASE mode...)
else
    $(error Invalid configuration: $(CONFIG). Use 'debug' or 'release')
endif

# Combined compiler flags
CXXFLAGS := $(CXXFLAGS_COMMON) $(CXXFLAGS_CONFIG)

# Include Directories
INCLUDE_DIRS := \
    -Iinclude \
    -Ilib/SDK/CHeaders/XPLM \
    -Ilib/SDK/CHeaders/Widgets \
    -Ilib/mavlink/c_library_v2 \
    -Ilib/mavlink/c_library_v2/common \
    -Ilib/mavlink/c_library_v2/all \
    -Ilib/simpleini \
    -Ilib/Eigen \
    -Ilib/XYZgeomag/src \
    -Iconfig

# Platform-specific Library Configuration
ifeq ($(OS),macos)
    LDFLAGS := -dynamiclib -undefined dynamic_lookup
    LDFLAGS += -F./lib/SDK/Libraries/Mac
    LIBS := -framework XPLM -framework XPWidgets
    LIBS += -framework OpenGL -framework GLUT
else ifeq ($(OS),linux)
    LDFLAGS := -shared -fPIC
    LDFLAGS += -L./lib/SDK/Libraries/Lin
    LIBS := -lXPLM -lXPWidgets -lGL -lglut -lpthread
endif

# Source Files (complete list)
SOURCES := \
    src/ConfigManager.cpp \
    src/configReader.cpp \
    src/ConnectionManager.cpp \
    src/DataRefManager.cpp \
    src/MAVLinkManager.cpp \
    src/px4xplane.cpp \
    src/TimeManager.cpp

# Object Files
OBJECTS := $(SOURCES:src/%.cpp=$(BUILD_DIR)/%.o)

# Dependencies
DEPS := $(OBJECTS:.o=.d)

# X-Plane plugin structure
PLUGIN_DIR := $(BUILD_DIR)/plugins/$(PROJECT_NAME)
TARGET := $(PLUGIN_DIR)/64/$(OUTPUT_NAME)
#==============================================================================
# Build Rules
#==============================================================================

.PHONY: all debug release clean distclean info help install check-deps

# Default target
all: check-deps $(TARGET)

# Debug build
debug:
	$(MAKE) CONFIG=debug

# Release build  
release:
	$(MAKE) CONFIG=release

# Create build directory
$(BUILD_DIR):
	@echo "Creating build directory: $(BUILD_DIR)"
	@mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	@echo "Compiling $< -> $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -MMD -MP -c $< -o $@

# Link the final plugin
$(TARGET): $(OBJECTS)
	@echo "Linking $(TARGET)"
	@mkdir -p $(dir $(TARGET))
	@mkdir -p $(PLUGIN_DIR)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)
	@if [ -f "config/config.ini" ]; then \
		cp config/config.ini $(PLUGIN_DIR)/; \
		echo "✓ Configuration copied: $(PLUGIN_DIR)/config.ini"; \
	fi
	@echo "✓ Build completed: $(TARGET)"
	@echo "✓ Plugin ready for X-Plane: $(PLUGIN_DIR)/"

# Include dependency files
-include $(DEPS)

#==============================================================================
# Utility Targets
#==============================================================================

# Clean build files
clean:
	@echo "Cleaning build files..."
	@rm -rf build/
	@echo "✓ Clean completed"

# Complete clean (including any generated files)
distclean: clean
	@echo "Performing deep clean..."
	@find . -name "*.tmp" -delete 2>/dev/null || true
	@find . -name "*.bak" -delete 2>/dev/null || true
	@echo "✓ Deep clean completed"

# Display build information
info:
	@echo "=== Build Information ==="
	@echo "Project: $(PROJECT_NAME) v$(VERSION)"
	@echo "OS: $(OS) ($(UNAME_S) $(UNAME_M))"
	@echo "Compiler: $(CXX)"
	@echo "Configuration: $(CONFIG)"
	@echo "Build Directory: $(BUILD_DIR)"
	@echo "Target: $(TARGET)"
	@echo "========================="

# Check dependencies and environment
check-deps:
	@echo "Checking build dependencies..."
	@if ! command -v $(CXX) >/dev/null 2>&1; then \
		echo "❌ ERROR: $(CXX) not found. Please install build tools."; \
		exit 1; \
	fi
	@if [ ! -d "lib/SDK" ]; then \
		echo "❌ ERROR: X-Plane SDK not found in lib/SDK/"; \
		echo "   Please ensure the X-Plane SDK is properly installed."; \
		exit 1; \
	fi
	@if [ ! -d "lib/mavlink" ]; then \
		echo "❌ ERROR: MAVLink library not found in lib/mavlink/"; \
		echo "   Please run: git submodule update --init --recursive"; \
		exit 1; \
	fi
	@if [ ! -d "lib/simpleini" ]; then \
		echo "❌ ERROR: SimpleINI library not found in lib/simpleini/"; \
		echo "   Please run: git submodule update --init --recursive"; \
		exit 1; \
	fi
	@if [ ! -d "lib/Eigen" ]; then \
		echo "❌ ERROR: Eigen library not found in lib/Eigen/"; \
		echo "   Please ensure Eigen is installed in lib/Eigen/"; \
		exit 1; \
	fi
	@echo "✓ All dependencies found"

# Install plugin to X-Plane (optional)
install: $(TARGET)
	@if [ -z "$(XPLANE_DIR)" ]; then \
		echo "❌ ERROR: XPLANE_DIR not set. Use: make install XPLANE_DIR=/path/to/xplane"; \
		exit 1; \
	fi
	@if [ ! -d "$(XPLANE_DIR)" ]; then \
		echo "❌ ERROR: X-Plane directory not found: $(XPLANE_DIR)"; \
		exit 1; \
	fi
	@echo "Installing plugin to X-Plane..."
	@mkdir -p "$(XPLANE_DIR)/Resources/plugins/$(PROJECT_NAME)/64"
	@cp $(TARGET) "$(XPLANE_DIR)/Resources/plugins/$(PROJECT_NAME)/64/"
	@if [ -f "config/config.ini" ]; then \
		cp config/config.ini "$(XPLANE_DIR)/Resources/plugins/$(PROJECT_NAME)/"; \
	fi
	@echo "✓ Plugin installed to: $(XPLANE_DIR)/Resources/plugins/$(PROJECT_NAME)/"

# Help target
help:
	@echo "=== px4xplane Build System ==="
	@echo ""
	@echo "Available targets:"
	@echo "  all         - Build the plugin (default: release)"
	@echo "  debug       - Build debug version"
	@echo "  release     - Build release version"
	@echo "  clean       - Remove build files"
	@echo "  distclean   - Complete clean"
	@echo "  info        - Show build information"
	@echo "  check-deps  - Verify dependencies"
	@echo "  install     - Install to X-Plane (requires XPLANE_DIR)"
	@echo "  help        - Show this help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make                     # Build release version"
	@echo "  make debug               # Build debug version"
	@echo "  make clean               # Clean build files"
	@echo "  make install XPLANE_DIR=/Applications/X-Plane 12"
	@echo ""
	@echo "Environment variables:"
	@echo "  CONFIG      - Build configuration (debug|release)"
	@echo "  XPLANE_DIR  - X-Plane installation directory"
	@echo "  CXX         - C++ compiler to use"
	@echo ""
	@echo "==============================="

#==============================================================================
# Advanced Features
#==============================================================================

# Parallel build support
.PARALLEL:

# Keep intermediate files for debugging
.SECONDARY: $(OBJECTS)

# Prevent make from deleting targets if interrupted
.DELETE_ON_ERROR:

# Make sure these targets aren't treated as files
.PHONY: all debug release clean distclean info help install check-deps