# px4xplane Build System Overview

This document provides a comprehensive overview of the px4xplane build system, which now supports **true cross-platform development** on Windows, Linux, and macOS.

## 🎯 Quick Reference

| Platform | Recommended Method | Command |
|----------|-------------------|---------|
| **All Platforms** | CMake (Unified) | `mkdir build && cd build && cmake .. && cmake --build .` |
| **Windows** | Visual Studio | Open `px4xplane.sln`, Build → Release\|x64 |
| **Linux** | Native Makefile | `make -f Makefile.linux` |
| **macOS** | Native Makefile | `make -f Makefile.macos` |

## 📁 Build System Files

### Core Build Files (Newly Added)

1. **CMakeLists.txt** ⭐ NEW - Recommended
   - **Purpose**: Unified cross-platform build configuration
   - **Supports**: Windows (MSVC/MinGW), Linux (GCC/Clang), macOS (Clang)
   - **Benefits**: Single build system for all platforms, IDE integration
   - **Output**: `build/{platform}/release/px4xplane/64/{platform}.xpl`

2. **Makefile.linux** ⭐ NEW
   - **Purpose**: Native Linux builds without CMake
   - **Compiler**: GCC 7+ or Clang 6+
   - **Benefits**: Direct control, no CMake dependency
   - **Output**: `build/linux/release/px4xplane/64/lin.xpl`

3. **Makefile.macos** ⭐ NEW
   - **Purpose**: Native macOS builds without CMake
   - **Compiler**: Apple Clang (Xcode Command Line Tools)
   - **Benefits**: Universal binary support (Intel + Apple Silicon)
   - **Output**: `build/macos/release/px4xplane/64/mac.xpl`

### Existing Build Files (Maintained)

4. **px4-xplane.sln + px4-xplane.vcxproj**
   - **Purpose**: Visual Studio project files
   - **Platform**: Windows only
   - **Benefits**: Full IDE integration, debugging support
   - **Output**: `build/windows/release/plugins/px4xplane/64/win.xpl`

## 🔨 Build Methods Comparison

### Method 1: CMake (Recommended for All Platforms)

**Pros:**
- ✅ Works on Windows, Linux, macOS
- ✅ Single build system to learn
- ✅ IDE integration (VS Code, CLion, Xcode, Visual Studio)
- ✅ Industry standard
- ✅ Easy for contributors

**Cons:**
- ⚠️ Requires CMake installation
- ⚠️ Extra abstraction layer

**When to use:**
- Cross-platform development
- Open-source contributions
- CI/CD pipelines
- Modern IDE workflows

**Example:**
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

---

### Method 2: Visual Studio (Windows Only)

**Pros:**
- ✅ Best Windows debugging experience
- ✅ Integrated profiler and tools
- ✅ No command-line required
- ✅ Point-and-click workflow

**Cons:**
- ⚠️ Windows only
- ⚠️ Large IDE installation
- ⚠️ Slower builds than command-line

**When to use:**
- Windows-only development
- Debugging complex issues
- Prefer GUI over command-line

**Example:**
1. Open `px4-xplane.sln` in Visual Studio
2. Select Release | x64
3. Press Ctrl+Shift+B

---

### Method 3: Native Makefiles (Linux/macOS)

**Pros:**
- ✅ No CMake dependency
- ✅ Direct compiler control
- ✅ Lightweight and fast
- ✅ Familiar to Unix developers

**Cons:**
- ⚠️ Separate files for each platform
- ⚠️ Manual maintenance required
- ⚠️ No IDE integration

**When to use:**
- Minimalist development environment
- Scripted builds
- Embedded Linux systems
- macOS Universal Binary builds

**Example (Linux):**
```bash
make -f Makefile.linux -j$(nproc)
```

**Example (macOS):**
```bash
make -f Makefile.macos -j$(sysctl -n hw.ncpu)
```

---

## 📦 Build Output Structure

All build methods produce a consistent X-Plane plugin structure:

```
px4xplane/
├── 64/
│   └── {platform}.xpl    # win.xpl, lin.xpl, or mac.xpl
├── config.ini            # Automatically copied
├── px4_params/           # PX4 parameter files (copied)
└── README.md             # Documentation
```

## 🛠️ Platform-Specific Details

### Windows

**Binary Output:** `win.xpl` (64-bit DLL)

**Required Libraries:**
- XPLM_64.lib (X-Plane SDK)
- XPWidgets_64.lib (X-Plane Widgets)
- Opengl32.lib (OpenGL)
- ws2_32.lib (Windows Sockets)

**Compiler Flags:**
- C++17 standard
- Unicode character set
- Platform defines: `IBM=1`, `WIN32`, `_WINDOWS`
- X-Plane SDK versions: `XPLM200=1` through `XPLM400=1`

**Optimizations (Release):**
- `/O2` (Maximum speed)
- `/GL` (Whole program optimization)
- `/LTCG` (Link-time code generation)

---

### Linux

**Binary Output:** `lin.xpl` (64-bit shared library)

**Required Libraries:**
- XPLM_64.so (X-Plane SDK)
- XPWidgets_64.so (X-Plane Widgets)
- libGL.so (OpenGL)
- libm.so (Math library)
- libpthread.so (POSIX threads)

**Compiler Flags:**
- C++17 standard
- `-fPIC` (Position-independent code)
- `-fvisibility=hidden` (Symbol visibility)
- Platform defines: `LIN=1`
- X-Plane SDK versions: `XPLM200=1` through `XPLM400=1`

**Linker Flags:**
- `-shared` (Create shared library)
- `-rdynamic` (Export symbols)
- `-nodefaultlibs` (No default libraries)

**Optimizations (Release):**
- `-O3` (Maximum optimization)

---

### macOS

**Binary Output:** `mac.xpl` (64-bit dynamic library, Universal Binary)

**Required Frameworks:**
- XPLM.framework (X-Plane SDK)
- XPWidgets.framework (X-Plane Widgets)
- OpenGL.framework (OpenGL)
- CoreFoundation.framework (macOS Core)

**Compiler Flags:**
- C++17 standard
- `-fPIC` (Position-independent code)
- `-fvisibility=hidden` (Symbol visibility)
- `-arch x86_64 -arch arm64` (Universal Binary)
- Platform defines: `APL=1`
- X-Plane SDK versions: `XPLM200=1` through `XPLM400=1`

**Linker Flags:**
- `-dynamiclib` (Create dynamic library)
- `-flat_namespace` (Flat symbol namespace)
- `-undefined suppress` (Allow undefined symbols - loaded by X-Plane)

**Optimizations (Release):**
- `-O3` (Maximum optimization)

**Universal Binary Support:**
The macOS build creates a Universal Binary that runs natively on both:
- Intel x86_64 processors
- Apple Silicon (ARM64) processors

---

## 🚀 Build Performance Tips

### Parallel Builds

**CMake:**
```bash
cmake --build . -j$(nproc)  # Linux/macOS
cmake --build . -j%NUMBER_OF_PROCESSORS%  # Windows
```

**Makefile:**
```bash
make -f Makefile.linux -j$(nproc)  # Linux
make -f Makefile.macos -j$(sysctl -n hw.ncpu)  # macOS
```

**Visual Studio:**
- Tools → Options → Projects and Solutions → Build and Run
- Set "maximum number of parallel project builds"

### Build Types

| Type | Optimizations | Debug Symbols | Use Case |
|------|---------------|---------------|----------|
| **Release** | Maximum (`-O3`/`/O2`) | Minimal | Production, final testing |
| **Debug** | None (`-O0`) | Full (`-g`) | Development, debugging |

### Incremental Builds

Only rebuild changed files:

**CMake:**
```bash
cmake --build .  # No clean needed
```

**Makefile:**
```bash
make -f Makefile.linux  # Only rebuilds changed files
```

**Visual Studio:**
- Build → Build Solution (not Rebuild)

### Clean Builds

Force complete rebuild:

**CMake:**
```bash
rm -rf build/*  # Or delete build/ folder
cmake .. && cmake --build .
```

**Makefile:**
```bash
make -f Makefile.linux clean
make -f Makefile.linux
```

**Visual Studio:**
- Build → Clean Solution
- Build → Rebuild Solution

---

## 🧪 Testing Your Build

### 1. Verify Plugin Binary

**Check file exists:**
```bash
# Windows
ls build/win/release/px4xplane/64/win.xpl

# Linux
ls build/lin/release/px4xplane/64/lin.xpl

# macOS
ls build/mac/release/px4xplane/64/mac.xpl
```

**Check file type:**
```bash
# Linux
file build/lin/release/px4xplane/64/lin.xpl
# Should show: ELF 64-bit LSB shared object, x86-64

# macOS
file build/mac/release/px4xplane/64/mac.xpl
# Should show: Mach-O 64-bit dynamically linked shared library

# Check Universal Binary (macOS)
lipo -info build/mac/release/px4xplane/64/mac.xpl
# Should show: x86_64 arm64
```

### 2. Install to X-Plane

**Manual installation:**
```bash
# Copy entire px4xplane folder to:
# Windows: X-Plane 12\Resources\plugins\
# Linux: ~/X-Plane 12/Resources/plugins/
# macOS: /Applications/X-Plane 12/Resources/plugins/
```

**Makefile installation:**
```bash
# Linux
make -f Makefile.linux install XPLANE_DIR=/path/to/X-Plane

# macOS
make -f Makefile.macos install XPLANE_DIR=/Applications/X-Plane\ 12
```

### 3. Verify Plugin Loads

1. Launch X-Plane
2. Check: Plugins → Plugin Admin
3. Look for **px4xplane** in the list
4. Menu bar should show **PX4-SITL**

### 4. Check Logs

**X-Plane Log.txt location:**
- Windows: `X-Plane 12\Log.txt`
- Linux: `~/X-Plane 12/Log.txt`
- macOS: `/Applications/X-Plane 12/Log.txt`

**Look for:**
```
Loaded: plugins/px4xplane/64/win.xpl (px4xplane v2.5.1)
```

**If errors:**
```
Plugin px4xplane failed to load: <error message>
```

---

## 📚 Additional Documentation

- **Quick Start Guide**: See [README.md](../README.md)
- **Detailed Build Instructions**: See [docs/BUILD.md](./docs/BUILD.md)
- **Custom Airframe Configuration**: See [docs/custom-airframe-config.md](./docs/custom-airframe-config.md)

---

## 🤝 Contributing Build System Improvements

When contributing to the build system:

1. **Test on all platforms** (Windows, Linux, macOS) if possible
2. **Maintain backward compatibility** with existing build methods
3. **Update documentation** (this file + docs/BUILD.md)
4. **Follow conventions:**
   - CMake: Modern CMake 3.15+ practices
   - Makefiles: POSIX-compliant where possible
   - Visual Studio: Portable paths only (no absolute paths)

---

## 📝 Version History

- **v2.5.1 (January 2025)**: Added CMake cross-platform support, Makefile.linux, Makefile.macos
- **v2.5.0 (January 2025)**: Professional cross-platform build system
- **v2.0.0 (2024)**: Visual Studio project with multi-platform support
- **v1.0.0 (2024)**: Initial release

---

**Repository**: https://github.com/alireza787b/px4xplane

**For Help**: https://github.com/alireza787b/px4xplane/issues
