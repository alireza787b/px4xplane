# px4xplane Build Guide

A comprehensive guide for building the **px4xplane** X-Plane plugin on Windows, macOS, and Linux systems.

**Repository**: [alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)

## Table of Contents

- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Build Methods](#build-methods)
  - [CMake (Recommended - Cross-Platform)](#cmake-recommended---cross-platform)
  - [Visual Studio (Windows)](#windows-build-visual-studio)
  - [Native Makefiles (macOS/Linux)](#macoslinux-build-makefile)
- [Configuration Options](#configuration-options)
- [Installation](#installation)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

## Quick Start

### CMake (Recommended - All Platforms)
```bash
# 1. Clone and prepare
git clone --recursive https://github.com/alireza787b/px4xplane.git
cd px4xplane

# 2. Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .

# 3. Your plugin is ready at:
# Windows: build/win/release/px4xplane/64/win.xpl
# macOS: build/mac/release/px4xplane/64/mac.xpl
# Linux: build/lin/release/px4xplane/64/lin.xpl
```

### Windows (Visual Studio)
```cmd
# 1. Clone and prepare
git clone https://github.com/alireza787b/px4xplane.git
cd px4xplane
git submodule update --init --recursive

# 2. Open in Visual Studio
# Double-click px4-xplane.sln or open via Visual Studio

# 3. Build
# Press Ctrl+Shift+B or Build > Build Solution

# 4. Your plugin is ready at:
# build/windows/release/plugins/px4xplane/64/win.xpl
```

### macOS/Linux (Native Makefiles)
```bash
# 1. Clone and prepare
git clone https://github.com/alireza787b/px4xplane.git
cd px4xplane
git submodule update --init --recursive

# 2. Build
# Linux:
make -f Makefile.linux

# macOS:
make -f Makefile.macos

# 3. Your plugin is ready at:
# Linux: build/linux/release/px4xplane/64/lin.xpl
# macOS: build/macos/release/px4xplane/64/mac.xpl
```

## Prerequisites

### System Requirements

**Windows:**
- Windows 10 or later (64-bit)
- Visual Studio 2019 or 2022 (Community/Professional/Enterprise)
- X-Plane 11 or 12

**macOS:**
- macOS 10.14 or later
- Xcode Command Line Tools
- X-Plane 11 or 12

**Linux:**
- Ubuntu 18.04+ / Debian 10+ / CentOS 7+ or equivalent
- GCC 7+ or Clang 6+
- X-Plane 11 or 12

### Required Dependencies

#### Install Build Tools

**Windows:**
1. **Install Visual Studio 2019 or 2022:**
   - Download from [Visual Studio Downloads](https://visualstudio.microsoft.com/downloads/)
   - Choose "Desktop development with C++" workload during installation
   - Ensure "MSVC v143 compiler toolset" and "Windows 10/11 SDK" are selected

**macOS:**
```bash
# Install Xcode Command Line Tools
xcode-select --install
```

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install build-essential g++ make
sudo apt install libgl1-mesa-dev libglu1-mesa-dev
```

**CentOS/RHEL/Fedora:**
```bash
# CentOS/RHEL
sudo yum groupinstall "Development Tools"
sudo yum install mesa-libGL-devel mesa-libGLU-devel

# Fedora
sudo dnf groupinstall "Development Tools"
sudo dnf install mesa-libGL-devel mesa-libGLU-devel
```

#### Project Dependencies

The following are automatically handled by the build system:

- **X-Plane SDK** - Already included in `lib/SDK/`
- **MAVLink v2** - Git submodule in `lib/mavlink/`
- **SimpleINI** - Git submodule in `lib/simpleini/`
- **Eigen** - Included in `lib/Eigen/`
- **XYZgeomag** - Git submodule in `lib/XYZgeomag/`

## Project Structure

```
px4xplane/
├── src/                    # Source code
│   ├── ConfigManager.cpp
│   ├── configReader.cpp
│   ├── ConnectionManager.cpp
│   ├── DataRefManager.cpp
│   ├── MAVLinkManager.cpp
│   ├── px4xplane.cpp
│   └── TimeManager.cpp
├── include/                # Header files
├── lib/                    # Dependencies
│   ├── SDK/               # X-Plane SDK
│   ├── mavlink/           # MAVLink library (submodule)
│   ├── simpleini/         # SimpleINI library (submodule)
│   ├── Eigen/             # Eigen math library
│   └── XYZgeomag/         # Geomag library (submodule)
├── config/                # Configuration files
├── build/                 # Build output (created during build)
│   ├── macos/            # macOS builds
│   └── linux/            # Linux builds
└── Makefile              # Build system
```

## Build Methods

### CMake (Recommended - Cross-Platform)

CMake is the **modern, unified build system** that works seamlessly on Windows, Linux, and macOS. This is the recommended method for all platforms.

#### Why CMake?

- ✅ **Single build system** for all platforms
- ✅ **Industry standard** for C++ cross-platform projects
- ✅ **Auto-detects** platform and configures correctly
- ✅ **IDE integration**: Works with VS Code, CLion, Xcode, Visual Studio
- ✅ **Easy for contributors**: Just `cmake . && make`

#### Step 1: Clone Repository

```bash
git clone --recursive https://github.com/alireza787b/px4xplane.git
cd px4xplane
```

If you already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

#### Step 2: Configure Build

**Linux/macOS:**
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

**Windows (Visual Studio):**
```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
```

**Windows (MinGW):**
```powershell
mkdir build
cd build
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
```

#### Step 3: Build

**All platforms:**
```bash
cmake --build . --config Release
```

Or use native build tools:
- **Linux/macOS:** `make`
- **Windows (VS):** Open `px4xplane.sln` in build directory
- **Windows (MinGW):** `mingw32-make`

#### Step 4: Locate Output

```
build/
├── win/release/px4xplane/    # Windows
├── lin/release/px4xplane/    # Linux
└── mac/release/px4xplane/    # macOS
    ├── 64/
    │   └── {platform}.xpl    # Plugin binary (win.xpl/lin.xpl/mac.xpl)
    ├── config.ini            # Configuration file
    └── README.md             # Documentation
```

#### CMake Build Options

**Custom compiler:**
```bash
cmake .. -DCMAKE_CXX_COMPILER=clang++
```

**Verbose build:**
```bash
cmake --build . --verbose
```

**Parallel build (faster):**
```bash
cmake --build . -j$(nproc)  # Linux/macOS
cmake --build . -j%NUMBER_OF_PROCESSORS%  # Windows
```

**Debug build:**
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build .
```

---

## Windows Build (Visual Studio)

### Initialize Project

**First time setup:**

1. **Clone the repository:**
   ```cmd
   git clone https://github.com/alireza787b/px4xplane.git
   cd px4xplane
   ```

2. **Initialize submodules:**
   ```cmd
   git submodule update --init --recursive
   ```

3. **Verify project structure:**
   ```cmd
   dir lib
   # Should show: Eigen, mavlink, SDK, simpleini, XYZgeomag
   ```

### Building with Visual Studio

#### Method 1: Visual Studio IDE (Recommended)

1. **Open the solution:**
   - Double-click `px4-xplane.sln`
   - Or: File → Open → Project/Solution → Select `px4-xplane.sln`

2. **Select configuration:**
   - **Release** (recommended for final use): Optimized, smaller file
   - **Debug** (for development): Debug symbols, easier debugging

3. **Build the project:**
   - Press `Ctrl+Shift+B`
   - Or: Build → Build Solution
   - Or: Right-click project → Build

4. **Monitor build progress:**
   ```
   Build started...
   1>------ Build started: Project: px4xplane, Configuration: Release x64 ------
   1>Compiling sources...
   1>Linking...
   1>Post-build: Copying configuration files
   1>Build completed: build/windows/release/win.xpl
   1>Plugin ready for X-Plane installation
   ========== Build: 1 succeeded, 0 failed, 0 up-to-date, 0 skipped ==========
   ```

#### Method 2: Command Line (MSBuild)

```cmd
# Open "Developer Command Prompt for VS 2022"

# Build Release version
msbuild px4-xplane.sln /p:Configuration=Release /p:Platform=x64

# Build Debug version
msbuild px4-xplane.sln /p:Configuration=Debug /p:Platform=x64

# Clean and rebuild
msbuild px4-xplane.sln /p:Configuration=Release /p:Platform=x64 /t:Clean,Build
```

### Build Output

After successful build:

**Release build:**
```
build/windows/release/
├── win.xpl              # X-Plane plugin (main file)
├── config.ini           # Configuration file (copied automatically)
└── win.pdb              # Debug symbols (for troubleshooting)
```

**Debug build:**
```
build/windows/debug/
├── win.xpl              # X-Plane plugin with debug info
├── config.ini           # Configuration file
└── win.pdb              # Debug symbols
```

### Project Configuration Details

The Visual Studio project is configured with:

**Include Directories:**
- `include/` - Project headers
- `lib/SDK/CHeaders/XPLM/` - X-Plane SDK XPLM
- `lib/SDK/CHeaders/Widgets/` - X-Plane SDK Widgets  
- `lib/mavlink/c_library_v2/` - MAVLink library
- `lib/simpleini/` - SimpleINI library
- `lib/Eigen/` - Eigen math library
- `lib/XYZgeomag/src/` - Geomagnetic library
- `config/` - Configuration headers

**Preprocessor Definitions:**
- `IBM=1` - Windows platform identifier
- `XPLM200=1` through `XPLM400=1` - X-Plane SDK versions
- `_CRT_SECURE_NO_WARNINGS` - Suppress MSVC warnings

**Libraries:**
- `XPLM_64.lib` - X-Plane SDK main library
- `XPWidgets_64.lib` - X-Plane SDK widgets
- `Opengl32.lib` - OpenGL graphics
- `ws2_32.lib` - Windows sockets

### Alternative: Windows Makefile (MSYS2/MinGW)

For developers who prefer command-line builds on Windows:

#### Setup MSYS2 Environment

1. **Install MSYS2:**
   - Download from [https://www.msys2.org/](https://www.msys2.org/)
   - Run the installer and follow setup instructions

2. **Install build tools:**
   ```bash
   # Open "MSYS2 MinGW 64-bit" terminal
   pacman -S mingw-w64-x86_64-gcc
   pacman -S mingw-w64-x86_64-make
   pacman -S git
   ```

3. **Build with Windows Makefile:**
   ```bash
   # Navigate to project directory
   cd /c/path/to/px4xplane
   
   # Use the Windows-specific Makefile
   make -f Makefile.windows
   
   # Or copy it as the main Makefile
   cp Makefile.windows Makefile
   make
   ```

**Note**: The Windows Makefile is provided separately for MSYS2/MinGW users. Most Windows developers should use Visual Studio as described above.

## macOS/Linux Build (Makefile)

### Initialize Submodules

**First time setup only:**
```bash
git submodule update --init --recursive
```

This downloads the required MAVLink, SimpleINI, and XYZgeomag libraries.

### Basic Build Commands

**Build release version (recommended):**
```bash
make
# or explicitly:
make release
```

**Build debug version:**
```bash
make debug
```

**Clean build files:**
```bash
make clean
```

**Complete clean:**
```bash
make distclean
```

### Build Output

After successful build, you'll find:

**macOS:**
```
build/macos/release/mac.xpl      # Release plugin
build/macos/debug/mac.xpl        # Debug plugin
```

**Linux:**
```
build/linux/release/linux.xpl   # Release plugin
build/linux/debug/linux.xpl     # Debug plugin
```

## Configuration Options

### Build Configurations

| Configuration | Description | Use Case |
|---------------|-------------|----------|
| `release` | Optimized (-O3), no debug symbols | Production use, best performance |
| `debug` | Debug symbols (-g), no optimization | Development, debugging |

### Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `CONFIG` | Build configuration | `CONFIG=debug make` |
| `CXX` | C++ compiler | `CXX=clang++ make` |
| `XPLANE_DIR` | X-Plane installation | See installation section |

### Advanced Build Options

**Use specific compiler:**
```bash
CXX=clang++ make
```

**Force debug build:**
```bash
CONFIG=debug make
```

**Parallel build (faster on multi-core systems):**
```bash
make -j$(nproc)    # Linux
make -j$(sysctl -n hw.ncpu)  # macOS
```

## Installation

### Manual Installation

#### Windows

1. **Build the plugin:**
   - Open `px4-xplane.sln` in Visual Studio
   - Build → Build Solution (Release configuration recommended)

2. **Locate X-Plane plugins directory:**
   ```
   X-Plane 12/Resources/plugins/
   # Example: C:\X-Plane 12\Resources\plugins\
   ```

3. **Create plugin directory:**
   ```cmd
   mkdir "C:\X-Plane 12\Resources\plugins\px4xplane\64"
   ```

4. **Copy plugin files:**
   ```cmd
   # Copy the plugin
   copy "build\windows\release\win.xpl" "C:\X-Plane 12\Resources\plugins\px4xplane\64\"
   
   # Copy configuration (optional)
   copy "config\config.ini" "C:\X-Plane 12\Resources\plugins\px4xplane\"
   ```

#### macOS

1. **Build the plugin:**
   ```bash
   make
   ```

2. **Create plugin directory:**
   ```bash
   mkdir -p "/Applications/X-Plane 12/Resources/plugins/px4xplane/64"
   ```

3. **Copy plugin files:**
   ```bash
   # Copy the plugin
   cp build/macos/release/mac.xpl "/Applications/X-Plane 12/Resources/plugins/px4xplane/64/"
   
   # Copy configuration (optional)
   cp config/config.ini "/Applications/X-Plane 12/Resources/plugins/px4xplane/"
   ```

#### Linux

1. **Build the plugin:**
   ```bash
   make
   ```

2. **Create plugin directory:**
   ```bash
   mkdir -p "/path/to/xplane/Resources/plugins/px4xplane/64"
   ```

3. **Copy plugin files:**
   ```bash
   # Copy the plugin
   cp build/linux/release/linux.xpl "/path/to/xplane/Resources/plugins/px4xplane/64/"
   
   # Copy configuration (optional)
   cp config/config.ini "/path/to/xplane/Resources/plugins/px4xplane/"
   ```

### Automatic Installation

Use the built-in install target:

**macOS:**
```bash
make install XPLANE_DIR="/Applications/X-Plane 12"
```

**Linux:**
```bash
make install XPLANE_DIR="/home/user/X-Plane-12"
```

## Troubleshooting

### Common Issues

#### Build Errors

**Windows: "XPLM_64.lib not found"**
- Ensure X-Plane SDK is in `lib/SDK/Libraries/Win/`
- Check that you're building for x64 platform
- Verify Visual Studio is configured for 64-bit builds

**Windows: "Cannot open include file 'mavlink/mavlink.h'"**
```cmd
# Initialize git submodules
git submodule update --init --recursive
```

**Windows: Build fails with "MSB8066: Custom build exited with code 1"**
- Close Visual Studio
- Delete `build/` folder
- Reopen Visual Studio and rebuild

**macOS/Linux: "Compiler not found"**
```bash
# Check if compiler is installed
which g++     # Linux
which clang++  # macOS

# Install build tools (see Prerequisites section)
```

**macOS/Linux: "X-Plane SDK not found"**
- Ensure `lib/SDK/` directory exists and contains X-Plane SDK files
- The SDK should have been included with the project

**macOS/Linux: "MAVLink library not found"**
```bash
# Initialize git submodules
git submodule update --init --recursive
```

**Permission denied during build**
```bash
# Make sure you have write permissions
chmod +w .
# Or run from a directory you own
```

#### Runtime Issues

**Plugin not loading in X-Plane:**

1. **Check X-Plane's Log.txt file for errors:**
   - Windows: `Documents/X-Plane 12/Log.txt`
   - macOS: `~/Desktop/X-Plane 12/Log.txt`  
   - Linux: `~/X-Plane 12/Log.txt`

2. **Verify plugin location:**
   ```
   X-Plane/Resources/plugins/px4xplane/64/
   ├── win.xpl     (Windows)
   ├── mac.xpl     (macOS)
   └── linux.xpl   (Linux)
   ```

3. **Check file permissions (macOS/Linux):**
   ```bash
   chmod +x /path/to/xplane/Resources/plugins/px4xplane/64/*.xpl
   ```

4. **Windows: Missing Visual C++ Redistributable:**
   - Download and install [Microsoft Visual C++ Redistributable](https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist)

**Connection issues with PX4:**
1. Check firewall settings (ports 14560, 14580)
2. Verify PX4 SITL is running
3. Check configuration in `config.ini`

### Debug Build

For debugging, use the debug configuration:

**Windows:**
- Select "Debug" configuration in Visual Studio
- Build the project
- Use Visual Studio debugger or attach to X-Plane process

**macOS/Linux:**
```bash
make debug
```

Debug builds include:
- Debug symbols for GDB/LLDB/Visual Studio
- No optimization (easier debugging)
- Additional runtime checks
- Detailed logging

### Getting Help

**Display build information:**
```bash
make info
```

**Check dependencies:**
```bash
make check-deps
```

**Show all available commands:**
```bash
make help
```

## Development

### Code Organization

- **src/**: All C++ source files
- **include/**: Header files
- **lib/**: External dependencies
- **config/**: Configuration files

### Development Workflow

#### Windows (Visual Studio)

1. **Make changes** to source code in Visual Studio
2. **Build debug version:**
   - Select "Debug" configuration
   - Press `Ctrl+Shift+B` or Build → Build Solution
3. **Test** in X-Plane
4. **Debug** using Visual Studio:
   - Set breakpoints in source code
   - Debug → Attach to Process → Select X-Plane
   - Or launch X-Plane from Visual Studio debugger
5. **Build release** when ready:
   - Select "Release" configuration  
   - Build → Build Solution

#### macOS/Linux (Makefile)

1. **Make changes** to source code
2. **Build debug version:**
   ```bash
   make debug
   ```
3. **Test** in X-Plane
4. **Debug** if needed using GDB/LLDB:
   ```bash
   # Attach to running X-Plane process
   gdb -p $(pidof X-Plane)
   # Or use LLDB on macOS
   lldb -p $(pgrep X-Plane)
   ```
5. **Build release** when ready:
   ```bash
   make release
   ```

### Adding New Source Files

#### Windows (Visual Studio)
1. **Add source file (.cpp):**
   - Right-click project in Solution Explorer
   - Add → New Item → C++ File (.cpp)
   - Or Add → Existing Item to add existing file

2. **Add header file (.h):**
   - Right-click project in Solution Explorer  
   - Add → New Item → Header File (.h)

3. **Rebuild:**
   - Build → Rebuild Solution

#### macOS/Linux (Makefile)
If you add new `.cpp` files:

1. Add the file path to `SOURCES` in the Makefile:
   ```makefile
   SOURCES := \
       src/ConfigManager.cpp \
       src/configReader.cpp \
       # ... existing files ...
       src/YourNewFile.cpp
   ```

2. Rebuild:
   ```bash
   make clean && make
   ```

### Performance Tips

- **Windows**: Use "Release" configuration for final testing (significant performance improvement)
- **All platforms**: Use parallel builds:
  - **Windows**: Build → Build Solution uses multiple cores automatically
  - **macOS/Linux**: `make -j$(nproc)` or `make -j$(sysctl -n hw.ncpu)`
- Use debug configuration only during development

### Best Practices

1. **Always clean** before important builds:
   - **Windows**: Build → Clean Solution, then Build → Rebuild Solution
   - **macOS/Linux**: `make clean && make`

2. **Test both configurations**: debug for development, release for final testing

3. **Version control**: 
   - Commit working code before major changes
   - Don't commit build artifacts (`build/` folders)
   - Keep submodules updated: `git submodule update --remote`

4. **Cross-platform compatibility**:
   - Test on target platforms when possible
   - Use relative paths in code
   - Follow C++17 standards

5. **Plugin installation**:
   - Always test in clean X-Plane installation
   - Verify plugin loads in X-Plane's Plugin Admin
   - Check Log.txt for any warnings or errors

### Continuous Integration

For automated builds across platforms, the project includes:
- **Windows**: Visual Studio project files
- **macOS/Linux**: Professional Makefiles  
- **Cross-platform**: Git submodules for dependencies

### Platform-Specific Notes

**Windows:**
- Output: `win.xpl` (64-bit only)
- Dependencies handled automatically by Visual Studio
- Use Windows-style paths in file operations

**macOS:**
- Output: `mac.xpl` (Universal or x86_64)
- Framework-based X-Plane SDK linking
- Use Unix-style paths

**Linux:**
- Output: `linux.xpl` (x86_64)
- Shared library-based X-Plane SDK linking  
- Use Unix-style paths

## Version Information

- **Build System Version**: 2.5.1 (Unified Cross-Platform with CMake)
- **Supported Platforms**:
  - Windows 10+ (Visual Studio 2019/2022, MinGW)
  - macOS 10.14+ (Xcode Command Line Tools, Universal Binary support)
  - Linux (Ubuntu 18.04+, CentOS 7+, Fedora, Arch)
- **Build Methods**:
  - **CMake**: Unified cross-platform (Recommended)
  - **Visual Studio**: Windows IDE/MSBuild
  - **Native Makefiles**: Makefile.linux, Makefile.macos
- **C++ Standard**: C++17
- **X-Plane SDK**: Latest version included (4.0.0+)
- **Plugin Version**: 2.5.2
- **Repository**: [alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)

## Additional Resources

- **X-Plane SDK Documentation**: [developer.x-plane.com](https://developer.x-plane.com/)
- **PX4 SITL Documentation**: [docs.px4.io](https://docs.px4.io/main/en/simulation/)
- **MAVLink Protocol**: [mavlink.io](https://mavlink.io/)
- **Project Issues**: [GitHub Issues](https://github.com/alireza787b/px4xplane/issues)

---

**Need Help?** 
- Check the troubleshooting section above
- Run `make help` for quick reference
- Review X-Plane's Log.txt for runtime issues