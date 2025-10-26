# px4xplane Versioning and Release Guide

**For Developers and Maintainers**

This guide explains how to create new versions and releases for the px4xplane project with automated CI/CD.

---

## Quick Reference

### Creating a New Release

```bash
# 1. Work on develop branch (no builds)
git checkout develop
# ... make changes, test locally ...

# 2. Merge to master (triggers builds but no release)
git checkout master
git merge develop
git push origin master

# 3. Update version numbers (see below)
# Edit: include/VersionInfo.h, CMakeLists.txt, CHANGELOG.md, README.md

# 4. Commit version changes
git add include/VersionInfo.h CMakeLists.txt CHANGELOG.md README.md
git commit -m "Release vX.Y.Z: Description"

# 5. Create and push version tag (triggers release)
git tag -a vX.Y.Z -m "Release vX.Y.Z: Description"
git push origin master
git push origin vX.Y.Z

# 6. GitHub Actions automatically creates release with binaries
# Monitor: https://github.com/alireza787b/px4xplane/actions
```

---

## Branch Strategy

### `develop` Branch
- **Purpose**: Active development, testing, experimentation
- **CI/CD**: **NO builds triggered** (saves GitHub Actions minutes)
- **Usage**:
  ```bash
  git checkout develop
  # Make changes, commit freely
  git push origin develop
  ```

### `master` Branch
- **Purpose**: Stable, tested code ready for release
- **CI/CD**: **Builds triggered on every push** (Windows, Linux, macOS)
- **Usage**:
  ```bash
  git checkout master
  git merge develop  # Merge when ready
  git push origin master  # Triggers builds, NO release
  ```

### Version Tags (`v*.*.*`)
- **Purpose**: Official releases
- **CI/CD**: **Triggers full release workflow** (build + GitHub Release creation)
- **Format**: `v{MAJOR}.{MINOR}.{PATCH}` (e.g., `v3.0.0`, `v3.1.0`, `v3.1.1`)
- **Usage**:
  ```bash
  git tag -a v3.0.0 -m "Release v3.0.0: Description"
  git push origin v3.0.0  # Triggers release
  ```

---

## Semantic Versioning

We follow [Semantic Versioning 2.0.0](https://semver.org/):

### MAJOR.MINOR.PATCH

- **MAJOR** (`X.0.0`): Breaking changes, incompatible API changes, structural changes
  - Example: v2.x.x → v3.0.0 (plugin structure reorganization)
  - Requires migration guide in CHANGELOG.md

- **MINOR** (`x.Y.0`): New features, backward-compatible additions
  - Example: v3.0.0 → v3.1.0 (new aircraft support, new UI features)
  - Reset BUILD to "001" for minor versions

- **PATCH** (`x.x.Z`): Bug fixes, backward-compatible changes
  - Example: v3.1.0 → v3.1.1 (EKF tuning, sensor noise fix)
  - Increment BUILD number

### Pre-Release Versions

- **Alpha**: `v3.1.0-alpha` (early testing, unstable)
- **Beta**: `v3.1.0-beta` (feature complete, testing)
- **Release Candidate**: `v3.1.0-rc1` (final testing before release)

Pre-release tags automatically mark GitHub Release as "pre-release".

---

## Step-by-Step Release Process

### Step 1: Update Version Numbers

Edit **4 files** with version information:

#### 1. `include/VersionInfo.h`

```cpp
constexpr const char* VERSION = "3.1.0";  // Update version
constexpr const char* PHASE = "Stable";    // Stable, Beta, Alpha
constexpr const char* BUILD = "001";       // Increment or reset
```

**Build Number Rules**:
- **MAJOR version**: Reset to "001"
- **MINOR version**: Reset to "001"
- **PATCH version**: Increment (e.g., "001" → "002")

#### 2. `CMakeLists.txt`

```cmake
project(px4xplane VERSION 3.1.0 LANGUAGES CXX)  # Update version
```

#### 3. `CHANGELOG.md`

Add new section at the top:

```markdown
## [3.1.0] - 2025-01-26

### Added
- New aircraft support: Cessna 208 Caravan
- Advanced autopilot UI with waypoint visualization

### Fixed
- EKF2 height estimate stability improvements
- GPS sensor noise tuning

### Changed
- Increased HIL_SENSOR rate to 250Hz

### Technical Details
- Files Modified: src/MAVLinkManager.cpp, config/px4_params/*
```

#### 4. `README.md`

Update "Latest Release" section:

```markdown
## Latest Release - Version 3.1.0 (January 2025)

### What's New in Version 3.1.0

- **🛩️ New Aircraft**: Cessna 208 Caravan support
- **🎮 Advanced Autopilot UI**: Waypoint visualization and mission planning
- **📊 EKF Improvements**: Better height estimate stability
```

### Step 2: Commit Version Changes

```bash
git add include/VersionInfo.h CMakeLists.txt CHANGELOG.md README.md
git commit -m "Release v3.1.0: New aircraft and autopilot UI

- Added Cessna 208 Caravan support
- Implemented advanced autopilot UI with waypoints
- Improved EKF2 height estimate stability
- Updated HIL_SENSOR rate to 250Hz

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>"
```

### Step 3: Create Annotated Tag

```bash
git tag -a v3.1.0 -m "Release v3.1.0: New aircraft and autopilot UI

Major improvements:
- Cessna 208 Caravan aircraft support
- Advanced autopilot UI with waypoint visualization
- EKF2 height estimate stability improvements
- HIL_SENSOR rate increased to 250Hz

See CHANGELOG.md for complete details."
```

### Step 4: Push to GitHub

```bash
# Push commit
git push origin master

# Push tag (triggers release)
git push origin v3.1.0
```

### Step 5: Monitor Release Build

1. **GitHub Actions**: https://github.com/alireza787b/px4xplane/actions
   - Watch "Release px4xplane" workflow
   - Build time: 5-15 minutes
   - Builds: Windows, Linux, macOS (parallel)

2. **GitHub Releases**: https://github.com/alireza787b/px4xplane/releases
   - Release created automatically
   - Binaries uploaded: `px4xplane-{platform}-v3.1.0.zip`

---

## Automated Release Workflow

### What Happens Automatically

1. **Tag Push Detection** → `release.yml` workflow triggers
2. **Create GitHub Release** → Empty release created with version tag
3. **Build All Platforms** (parallel):
   - **Windows**: MSVC 2022, x64, `win.xpl`
   - **Linux**: Ubuntu 22.04, GCC, x64, `lin.xpl`
   - **macOS**: macOS 14, Clang, Universal Binary (Intel + Apple Silicon), `mac.xpl`
4. **Package Binaries** → ZIP files with complete plugin structure:
   ```
   px4xplane/
   ├── 64/
   │   ├── {platform}.xpl
   │   └── config.ini
   ├── px4_airframes/
   │   ├── 5001_xplane_cessna172
   │   ├── 5002_xplane_tb2
   │   ├── 5010_xplane_ehang184
   │   ├── 5020_xplane_alia250
   │   └── 5021_xplane_qtailsitter
   └── README.md
   ```
5. **Upload to Release** → All 3 platform ZIPs attached to GitHub Release
6. **Release Summary** → Workflow posts success/failure status

### Release Workflow Files

- **`.github/workflows/build.yml`**: CI builds on master push (no release)
- **`.github/workflows/release.yml`**: Full release on version tag push

---

## Common Scenarios

### Scenario 1: Bug Fix Release (Patch)

```bash
# Current version: v3.1.0
# Fix a critical bug

# 1. Update version numbers
# VersionInfo.h: VERSION = "3.1.1", BUILD = "002"
# CMakeLists.txt: VERSION 3.1.1
# CHANGELOG.md: Add [3.1.1] section
# README.md: Update "Latest Release"

# 2. Commit and tag
git commit -m "Release v3.1.1: Fix critical EKF reset issue"
git tag -a v3.1.1 -m "Release v3.1.1: Fix critical EKF reset issue"
git push origin master && git push origin v3.1.1
```

### Scenario 2: New Feature Release (Minor)

```bash
# Current version: v3.1.1
# Add new aircraft support

# 1. Update version numbers
# VersionInfo.h: VERSION = "3.2.0", BUILD = "001" (reset)
# CMakeLists.txt: VERSION 3.2.0
# CHANGELOG.md: Add [3.2.0] section
# README.md: Update "Latest Release"

# 2. Commit and tag
git commit -m "Release v3.2.0: Add Cessna 208 aircraft support"
git tag -a v3.2.0 -m "Release v3.2.0: Add Cessna 208 aircraft support"
git push origin master && git push origin v3.2.0
```

### Scenario 3: Breaking Changes (Major)

```bash
# Current version: v3.2.0
# Major architectural change

# 1. Update version numbers
# VersionInfo.h: VERSION = "4.0.0", BUILD = "001" (reset)
# CMakeLists.txt: VERSION 4.0.0
# CHANGELOG.md: Add [4.0.0] section with BREAKING CHANGES and migration guide
# README.md: Add v4.0.0 with migration instructions

# 2. Commit and tag
git commit -m "Release v4.0.0: Major architecture overhaul"
git tag -a v4.0.0 -m "Release v4.0.0: Major architecture overhaul"
git push origin master && git push origin v4.0.0
```

---

## Testing Releases Locally

### Test Build Before Release

```bash
# Test Windows build (Visual Studio)
# Open px4-xplane.sln, Build → Build Solution

# Test Linux build
make -f Makefile.linux

# Test macOS build
make -f Makefile.macos

# Test CMake build (cross-platform)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

### Verify Plugin Structure

```bash
# Check output structure matches expected format
ls -R build/win/Release/px4xplane/
# Expected:
# 64/win.xpl
# 64/config.ini
# px4_airframes/5001_xplane_cessna172
# px4_airframes/5002_xplane_tb2
# ...
```

---

## Troubleshooting

### Release Build Failed

1. Check GitHub Actions logs: https://github.com/alireza787b/px4xplane/actions
2. Look for specific platform failure (Windows/Linux/macOS)
3. Test locally with same build commands as workflow
4. Fix issue, commit, and create new patch version

### Release Created But Missing Binaries

1. Check "build-and-upload" job logs in workflow
2. Verify ZIP packaging step succeeded
3. Check "Upload release asset" step for errors
4. Manually upload binaries if needed (temporary workaround)

### Wrong Version Number in Release

1. Delete tag locally: `git tag -d v3.1.0`
2. Delete tag remotely: `git push --delete origin v3.1.0`
3. Delete GitHub Release manually
4. Fix version numbers, commit, re-tag

---

## Best Practices

✅ **DO**:
- Test builds locally before tagging
- Update all 4 version files consistently
- Write clear, descriptive CHANGELOG entries
- Use annotated tags (`-a` flag) with detailed messages
- Follow semantic versioning strictly
- Test on actual X-Plane installation before release

❌ **DON'T**:
- Tag directly on develop branch (always merge to master first)
- Reuse version numbers (each version is immutable)
- Skip CHANGELOG updates
- Create release without testing builds
- Use lightweight tags (always use `-a`)

---

## GitHub Actions CI/CD Reference

### Workflow Triggers

| Event | Branch/Tag | Workflow | Result |
|-------|-----------|----------|--------|
| Push | `develop` | None | No builds (saves minutes) |
| Push | `master` | `build.yml` | Build all platforms, NO release |
| Push | `v*.*.*` | `release.yml` | Build + Create GitHub Release |
| Manual | Any | Both | Can trigger manually from Actions tab |

### Build Matrix

| Platform | OS | Compiler | Architecture | Output |
|----------|----|----|--------------|--------|
| Windows | windows-2022 | MSVC 2022 | x64 | win.xpl |
| Linux | ubuntu-22.04 | GCC 11 | x64 | lin.xpl |
| macOS | macos-14 | Clang 15 | Universal (x64 + ARM64) | mac.xpl |

---

## Contact

For questions about versioning or CI/CD:
- **GitHub Issues**: https://github.com/alireza787b/px4xplane/issues
- **Documentation**: https://github.com/alireza787b/px4xplane

---

**Last Updated**: 2025-01-26 (px4xplane v3.0.0)
