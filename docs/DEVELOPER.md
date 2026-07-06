# Developer Guide

**Quick reference for px4xplane development workflow, versioning, and CI/CD.**

---

## Quick Start

### Development Workflow

```bash
# 1. Create feature branch (optional but recommended)
git checkout -b feature/my-feature

# 2. Make changes and test locally
# Build: see docs/BUILD.md

# 3. Commit changes
git add .
git commit -m "Add feature: description"

# 4. Push to master
git checkout master
git merge feature/my-feature
git push origin master
# → No release is created. Use workflow_dispatch for a manual CI build.

# 5. When ready to release:
# Update version numbers and public docs (see Versioning below)
git add CHANGELOG.md CMakeLists.txt Makefile.linux Makefile.macos \
  include/VersionInfo.h docs/config-editor.html README.md docs/index.md \
  config/config.ini docs/*_XPLANE12_TEST.md aircraft/QuadTailsitter/README.md
git commit -m "Release vX.Y.Z: description"
git tag -a vX.Y.Z -m "Release vX.Y.Z: description"
git push origin master && git push origin vX.Y.Z
# → Automatic GitHub Release with binaries
```

---

## Versioning

### Version Format: `MAJOR.MINOR.PATCH`

- **MAJOR** (x.0.0): Breaking changes, migration required
- **MINOR** (x.1.x): New features, backward compatible
- **PATCH** (x.x.1): Bug fixes, backward compatible

### Files to Update

**1. `include/VersionInfo.h`**
```cpp
constexpr const char* VERSION = "4.1.0";  // Update
constexpr const char* BUILD = "001";      // Reset for major/minor, increment for patch
```

**2. `CMakeLists.txt`**
```cmake
project(px4xplane VERSION 4.1.0 LANGUAGES CXX)  // Update
```

**3. `Makefile.linux` and `Makefile.macos`**
```make
VERSION := 4.1.0
```

**4. `docs/config-editor.html`** - Update the displayed editor version/build.

**5. Public package docs** - Update any current-package references in
`README.md`, `docs/index.md`, `config/config.ini`, aircraft docs, and
flight-test cards.

**6. `CHANGELOG.md`** - Add new section at top:
```markdown
## [4.1.0] - 2026-XX-XX

### Added
- Feature description

### Fixed
- Bug fix description
```

### Creating a Release

```bash
# 1. Update versions and public current-package references
# 2. Commit
git add CHANGELOG.md CMakeLists.txt Makefile.linux Makefile.macos \
  include/VersionInfo.h docs/config-editor.html README.md docs/index.md \
  config/config.ini docs/*_XPLANE12_TEST.md aircraft/QuadTailsitter/README.md
git commit -m "Release v4.1.0: Feature name"

# 3. Tag and push
git tag -a v4.1.0 -m "Release v4.1.0: Feature name"
git push origin master
git push origin v4.1.0

# 4. Monitor release
# → https://github.com/alireza787b/px4xplane/actions
# → Builds take 5-15 minutes
# → Release created automatically with 3 platform binaries
```

---

## CI/CD (GitHub Actions)

### Workflow Triggers

| Trigger | Branch/Tag | Result |
|---------|-----------|--------|
| Pull request to `master` | - | Build all platforms for validation |
| Push tag `v*.*.*` | - | Build + Create GitHub Release |
| Manual workflow dispatch in `build.yml` | Any | Build all platforms on demand |

### Build Outputs

**CI artifacts** (90-day retention):
- `px4xplane-windows.zip` (x64)
- `px4xplane-linux.zip` (x64)
- `px4xplane-macos.zip` (Universal: Intel + Apple Silicon)

**Release assets** are versioned:
- `px4xplane-windows-vX.Y.Z.zip`
- `px4xplane-linux-vX.Y.Z.zip`
- `px4xplane-macos-vX.Y.Z.zip`

**Structure** (identical on all platforms):
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

### Workflow Files

- `.github/workflows/build.yml` - Pull-request and manual build validation
- `.github/workflows/release.yml` - Release automation (builds + GitHub Release on tag)

---

## Common Tasks

### Bug Fix (Patch Release)

```bash
# Example: v4.0.1 -> v4.0.2

# 1. Fix bug and test
# 2. Update version surfaces listed above
# 3. Confirm no stale current version remains:
rg "4\\.0\\.1|v4\\.0\\.1|build 001"

# 4. Release
git commit -m "Release v4.0.2: Fix critical bug"
git tag -a v4.0.2 -m "Release v4.0.2: Fix critical bug"
git push origin master && git push origin v4.0.2
```

### New Feature (Minor Release)

```bash
# Example: v4.0.x -> v4.1.0

# 1. Implement feature and test
# 2. Update version surfaces listed above

# 3. Release
git commit -m "Release v4.1.0: New feature"
git tag -a v4.1.0 -m "Release v4.1.0: New feature"
git push origin master && git push origin v4.1.0
```

### Breaking Change (Major Release)

```bash
# Current: v4.1.0 -> Target: v5.0.0

# 1. Implement breaking change
# 2. Document migration in CHANGELOG.md (BREAKING CHANGES section)
# 3. Update version surfaces listed above

# 4. Release
git commit -m "Release v5.0.0: Breaking change description"
git tag -a v5.0.0 -m "Release v5.0.0: Breaking change description"
git push origin master && git push origin v5.0.0
```

---

## Testing Locally

### Build All Platforms

**Windows (Visual Studio):**
```cmd
# Open px4-xplane.sln
# Build → Build Solution (Release | x64)
```

**Linux:**
```bash
make -f Makefile.linux
# or
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
```

**macOS:**
```bash
make -f Makefile.macos
# or
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
```

### Verify Plugin Structure

```bash
# Windows
tree /F build\windows\release\plugins\px4xplane

# Linux/macOS
tree build/{platform}/Release/px4xplane
```

**Expected structure:**
- `64/{platform}.xpl` ✓
- `64/config.ini` ✓
- `px4_airframes/500*_xplane_*` ✓

---

## Troubleshooting

### Build Fails in CI/CD

1. Check GitHub Actions logs: https://github.com/alireza787b/px4xplane/actions
2. Identify failing platform (Windows/Linux/macOS)
3. Test locally on that platform
4. Fix and push

### Release Created But Missing Binaries

1. Check "build-and-upload" job in workflow
2. Look for ZIP packaging errors
3. Verify artifact upload step succeeded
4. If needed, manually upload ZIPs to release

### Wrong Version Number

```bash
# Delete tag locally and remotely
git tag -d v4.1.0
git push --delete origin v4.1.0

# Delete GitHub Release manually
# Fix version numbers, recommit, re-tag
```

---

## Best Practices

✅ **DO:**
- Test locally before pushing to master
- Update all version and public current-package references consistently
- Write clear CHANGELOG entries
- Use annotated tags (`git tag -a`)
- Test plugin in X-Plane before release

❌ **DON'T:**
- Push unfinished features to master
- Reuse version numbers
- Skip CHANGELOG updates
- Use lightweight tags
- Release without local testing

---

## Documentation Structure

```
px4xplane/
├── README.md              # User guide, installation, features
├── CHANGELOG.md           # Version history
├── docs/
│   ├── DEVELOPER.md       # This file: workflow, versioning, CI/CD
│   ├── BUILD.md           # Detailed build instructions (all platforms)
│   ├── BUILD_SYSTEM.md    # Technical build system details
│   ├── config-editor.html # Local config editor
│   ├── custom-airframe-config.md
│   └── developer/config-schema.md
```

---

## Links

- **Repository**: https://github.com/alireza787b/px4xplane
- **Issues**: https://github.com/alireza787b/px4xplane/issues
- **Releases**: https://github.com/alireza787b/px4xplane/releases
- **Actions**: https://github.com/alireza787b/px4xplane/actions
- **Official PX4**: https://github.com/PX4/PX4-Autopilot
- **PX4 X-Plane SITL merge**: https://github.com/PX4/PX4-Autopilot/pull/22493

---

**Last Updated**: 2026-07-06 (px4xplane v4.1.0)
