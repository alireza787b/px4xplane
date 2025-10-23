# GitHub Actions CI/CD Guide for px4xplane

## Overview

px4xplane uses **GitHub Actions** for automated builds and releases. This guide explains the complete workflow for developers and AI assistants working on this project.

---

## 🤖 Quick Reference for AI Assistants

**If you're an AI helping with this project, here's what you need to know:**

### Build Strategy
- **develop branch**: NO builds (saves GitHub minutes)
- **master branch**: Builds run automatically → Artifacts (90-day retention)
- **Version tags (v2.5.3)**: Builds run + Release created automatically

### Key Files
- **Workflows**: `.github/workflows/build.yml`, `.github/workflows/release.yml`
- **Version**: `include/VersionInfo.h` (update VERSION and BUILD)
- **Build systems**: `CMakeLists.txt`, `Makefile.linux`, `Makefile.macos`

### Release Process (Manual Versioning)
1. User updates `VersionInfo.h` with new version
2. Commit to master
3. Create tag: `git tag v2.5.3 && git push origin v2.5.3`
4. GitHub Actions automatically creates release with binaries

### Where Builds Are Stored
- **Artifacts** (temporary): https://github.com/alireza787b/px4xplane/actions
- **Releases** (permanent): https://github.com/alireza787b/px4xplane/releases

### Important Notes
- ❌ Do NOT build on develop branch (intentional)
- ✅ Versioning is manual (user controls when to release)
- ✅ Release creation is automatic (triggered by tags)
- ✅ Binary uploads are automatic (no manual upload needed)

---

## Workflow Strategy (IMPORTANT - Read This First!)

### Branch Strategy

```
develop branch
  ↓ (daily work, experiments, drafts)
  ↓ NO BUILDS RUN (saves GitHub minutes)
  ↓
  ↓ When ready to test/release
  ↓
master branch
  ↓ BUILDS RUN (Windows, Linux, macOS)
  ↓ Artifacts created (90-day retention)
  ↓
  ↓ When ready for official release
  ↓ Update VersionInfo.h + push tag v2.5.3
  ↓
GitHub Release
  ✓ RELEASE CREATED AUTOMATICALLY
  ✓ BINARIES UPLOADED AUTOMATICALLY
  ✓ PERMANENT STORAGE
```

### What Triggers Builds?

| Event | Build? | Artifacts? | Release? |
|-------|--------|-----------|----------|
| Push to `develop` | ❌ No | ❌ No | ❌ No |
| Push to `master` | ✅ Yes | ✅ Yes (90 days) | ❌ No |
| PR to `master` | ✅ Yes | ✅ Yes (90 days) | ❌ No |
| Push tag `v*.*.*` | ✅ Yes | ✅ Yes (90 days) | ✅ YES |

## Complete Developer Workflow

### Phase 1: Daily Development (No Builds)

**Branch:** `develop`

```bash
# Work on develop branch
git checkout develop

# Make changes, commit
git add .
git commit -m "Fix altitude sensor noise"
git push origin develop
```

**Result:**
- ✅ Code backed up to GitHub
- ❌ No builds run (saves GitHub minutes)
- ✅ Safe to experiment, draft, test locally

**Use Case:** Daily work, experiments, incomplete features

---

### Phase 2: Ready to Test Build

**Branch:** `develop` → `master`

```bash
# Merge to master to trigger builds
git checkout master
git merge develop
git push origin master
```

**Result:**
- ✅ **Builds run automatically** (Windows, Linux, macOS)
- ✅ **Artifacts available** at https://github.com/alireza787b/px4xplane/actions
- ✅ Artifacts kept for 90 days
- ❌ No release created yet

**Where are the binaries?**
1. Go to: https://github.com/alireza787b/px4xplane/actions
2. Click latest "Build px4xplane" workflow run
3. Scroll to "Artifacts" section
4. Download:
   - `px4xplane-windows`
   - `px4xplane-linux`
   - `px4xplane-macos`

**Use Case:** Test merged code before creating official release

---

### Phase 3: Create Official Release

**Branch:** `master` + version tag

```bash
# STEP 1: Update version in code
# Edit include/VersionInfo.h:
#   constexpr const char* VERSION = "2.5.3";  // ← Change version
#   constexpr const char* BUILD = "032";      // ← Increment build

# STEP 2: Commit version bump
git add include/VersionInfo.h
git commit -m "Release v2.5.3: Description of changes"
git push origin master

# STEP 3: Create and push tag
git tag v2.5.3
git push origin v2.5.3
```

**Result:**
- ✅ **Release created AUTOMATICALLY**
- ✅ **Binaries uploaded AUTOMATICALLY** (Windows, Linux, macOS ZIPs)
- ✅ **Changelog generated AUTOMATICALLY** from commit messages
- ✅ **Permanent storage** (never deleted)

**Where is the release?**
- Go to: https://github.com/alireza787b/px4xplane/releases
- Find release: "px4xplane v2.5.3"
- Download ZIPs:
  - `px4xplane-windows-v2.5.3.zip`
  - `px4xplane-linux-v2.5.3.zip`
  - `px4xplane-macos-v2.5.3.zip`

**Use Case:** Official release for end users

---

## What Gets Built Automatically

### On Every Push to Master
- ✅ Windows build (Visual Studio 2022, x64)
- ✅ Linux build (Ubuntu 22.04, GCC 11, x64)
- ✅ macOS build (Universal Binary: Intel x86_64 + Apple Silicon ARM64)
- ✅ Build artifacts (temporary, 90 days)

### On Version Tags (v*.*.*)
- ✅ All platform builds
- ✅ Automatic GitHub Release creation
- ✅ ZIP packages for each platform uploaded to release
- ✅ Changelog generated from commits
- ✅ Permanent storage

---

## Checking Build Status

### Method 1: README Badge
The README shows live build status:
- ![Build Passing](https://img.shields.io/badge/build-passing-brightgreen) = All good
- ![Build Failing](https://img.shields.io/badge/build-failing-red) = Something broke

### Method 2: Actions Tab
Visit: https://github.com/alireza787b/px4xplane/actions

You'll see:
- ✅ Green checkmark = Build passed
- ❌ Red X = Build failed
- 🟡 Yellow circle = Build in progress

### Method 3: Email Notifications
GitHub will email you if a build fails on your commit.

---

## Downloading Build Artifacts

### For Development/Testing

1. Go to: https://github.com/alireza787b/px4xplane/actions
2. Click on the latest successful build
3. Scroll down to "Artifacts"
4. Download:
   - `px4xplane-windows`
   - `px4xplane-linux`
   - `px4xplane-macos`

**Note:** Artifacts are kept for 90 days.

### For End Users (Releases)

1. Go to: https://github.com/alireza787b/px4xplane/releases
2. Download the ZIP for your platform
3. Extract and copy to X-Plane plugins folder

---

## Creating a Release

### Automatic Release (Recommended)

When you're ready to release a new version:

```bash
# 1. Update version in code (if needed)
#    - include/VersionInfo.h: VERSION = "2.5.3"
#    - CMakeLists.txt: project(px4xplane VERSION 2.5.3)
#    - Makefile.linux: VERSION := 2.5.3
#    - Makefile.macos: VERSION := 2.5.3

# 2. Commit changes
git add -A
git commit -m "Bump version to 2.5.3"
git push origin master

# 3. Create and push tag
git tag v2.5.3
git push origin v2.5.3

# 4. GitHub Actions automatically:
#    - Builds all platforms
#    - Creates GitHub Release
#    - Uploads binaries
#    - Generates changelog
```

**That's it!** Check https://github.com/alireza787b/px4xplane/releases in ~10-15 minutes.

### Manual Trigger (Testing)

To test the build workflow without pushing code:

1. Go to: https://github.com/alireza787b/px4xplane/actions
2. Click "Build px4xplane" workflow
3. Click "Run workflow" button
4. Select branch (usually `master`)
5. Click green "Run workflow" button

---

## Understanding the Workflows

### `build.yml` - Main CI/CD

**Triggers:**
- Push to `master` or `main` branch
- Pull requests
- Manual trigger

**What it does:**
1. Checks out code with submodules
2. Sets up build environment (MSVC/GCC/Clang)
3. Configures CMake
4. Builds plugin
5. Verifies .xpl file exists
6. Uploads artifacts

**Build time:** ~5 minutes (Windows), ~3 minutes (Linux), ~5 minutes (macOS)

### `release.yml` - Automated Releases

**Triggers:**
- Push tags matching `v*.*.*` (e.g., v2.5.2)
- Manual trigger

**What it does:**
1. Creates GitHub Release
2. Generates changelog from git commits
3. Builds all platforms
4. Packages each build into ZIP
5. Uploads ZIPs to release

**Build time:** ~15 minutes total

---

## Build Matrix Explained

```yaml
strategy:
  fail-fast: false
  matrix:
    include:
      - os: windows-2022
        platform: windows

      - os: ubuntu-22.04
        platform: linux

      - os: macos-14
        platform: macos
```

This means GitHub spins up **3 separate virtual machines** and builds in parallel:
- **Windows Server 2022** VM
- **Ubuntu 22.04** VM
- **macOS 14** (Apple Silicon + Intel via Rosetta)

**Cost:** Free for public repositories (2,000 minutes/month)

---

## Troubleshooting Builds

### Build Failed on Windows
**Check:**
- Visual Studio 2022 available?
- X-Plane SDK Windows libraries (`.lib` files) present?
- CMake configuration errors?

**View logs:** Click on failed job → Expand failed step

### Build Failed on Linux
**Check:**
- OpenGL development libraries installed?
- GCC version compatible (7+)?
- Case-sensitive include issues? (ConfigReader.h vs configReader.h)

### Build Failed on macOS
**Check:**
- Xcode Command Line Tools available?
- X-Plane SDK macOS frameworks present?
- Universal Binary linking working?

**Tip:** macOS builds can sometimes timeout on GitHub's shared runners. Re-run if this happens.

---

## Best Practices

### Before Pushing Code

```bash
# Test locally first!
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
# Verify build works

# Then push
git push origin master
```

### Version Numbering

Follow **Semantic Versioning** (semver):
- **v2.5.2** = MAJOR.MINOR.PATCH
- **MAJOR** (2): Breaking changes
- **MINOR** (5): New features, backward compatible
- **PATCH** (2): Bug fixes

### Commit Messages

Good commit messages help generate useful changelogs:

```bash
# Good ✅
git commit -m "Fix altitude drift in EKF2 sensor fusion"
git commit -m "Add Universal Binary support for macOS"

# Bad ❌
git commit -m "fix bug"
git commit -m "updates"
```

---

## GitHub Actions Limits (Free Tier)

### Monthly Quota (Resets Every Month)

| Resource | Free Limit | Resets When | Your Usage (Typical) |
|----------|-----------|-------------|---------------------|
| **Build minutes** | 2,000/month | 1st of each month | ~150-300 (10-20 builds to master) |
| **Concurrent jobs** | 20 | N/A | 3 (win + linux + mac) |
| **Storage** | 500 MB | N/A (total) | ~50 MB (artifacts) |
| **Artifact retention** | 90 days | Configurable | 90 days ✅ |

**Platform Multipliers:**
- Windows: 1× (5 min build = 5 min charged)
- Linux: 1× (3 min build = 3 min charged)
- macOS: 10× (5 min build = 50 min charged)

**Total per master build:** ~15 real minutes = ~60 charged minutes (due to macOS 10× multiplier)

### Why develop Branch Doesn't Build

**Savings Example:**
- Push to `develop` (no build): **0 minutes** ✅
- Push to `master` (builds): **~60 minutes** (charged)

**If we built on both branches:**
- Daily push to develop: 60 min
- Weekly merge to master: 60 min
- **Total waste:** ~1,800 min/month on develop builds alone

**Current strategy:**
- Only build on master: ~300-600 min/month
- **Plenty of headroom** (30% of quota)

### You Have Plenty of Budget

Even with daily merges to master, you'll use only ~30-50% of your monthly quota.

---

## Advanced: Modifying Workflows

### Add a New Build Step

Edit `.github/workflows/build.yml`:

```yaml
- name: My custom step
  run: |
    echo "Running custom build step..."
    ./my-script.sh
```

### Change Artifact Retention

```yaml
- uses: actions/upload-artifact@v4
  with:
    retention-days: 30  # Change from 90 to 30 days
```

### Add Build Notifications

Integrate with Slack/Discord/Email:

```yaml
- name: Notify on failure
  if: failure()
  uses: some-notification-action@v1
```

---

## Viewing Build Logs

1. Go to: https://github.com/alireza787b/px4xplane/actions
2. Click on a workflow run
3. Click on a job (e.g., "Build windows")
4. Expand steps to see detailed logs

**Tip:** Look for:
- ✓ Green checkmarks = Step passed
- ❌ Red X = Step failed
- Yellow warnings = Non-critical issues

---

## FAQ

### Q: Why did my build fail?
**A:** Click on the failed job and read the logs. Common issues:
- Missing dependencies
- Syntax errors in CMakeLists.txt
- Case-sensitive filename mismatches

### Q: Can I build locally without GitHub Actions?
**A:** Yes! Follow the instructions in [docs/BUILD.md](BUILD.md)

### Q: How do I skip a CI build?
**A:** Add `[skip ci]` to your commit message:
```bash
git commit -m "Update README [skip ci]"
```

### Q: Can I test PRs from contributors?
**A:** Yes! GitHub Actions automatically builds PRs.

### Q: How do I see artifact size?
**A:** Artifacts section shows file sizes after upload.

---

## Additional Resources

- **GitHub Actions Docs:** https://docs.github.com/en/actions
- **Workflow Syntax:** https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions
- **Action Marketplace:** https://github.com/marketplace?type=actions

---

## Status

**Current Workflows:**
- ✅ `build.yml` - CI/CD builds on every push
- ✅ `release.yml` - Automated releases on version tags

**Badges:**
- Build status: ![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)
- Latest release: ![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)

**Next Steps:**
- Consider adding automated testing
- Add code coverage reports
- Integrate with Discord/Slack notifications

---

**Need Help?** Open an issue at https://github.com/alireza787b/px4xplane/issues
