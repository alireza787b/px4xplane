# Developer Guide

Current package: `v4.2.1`. Changes for PR21 belong under `Unreleased`; this
work does not request a version bump, tag or release.

Normal setup is in the [Quick Start](../README.md#quick-start) and optional
[guided launcher guide](guided-launcher.md). Keep beginner instructions there
separate from experimental validation.

## Development Workflow

1. Work on a feature branch and keep changes scoped to the task.
2. Build and test locally using the [Build Guide](BUILD.md).
3. Add focused tests and an `Unreleased` changelog entry for behavioral changes.
4. Open a pull request against `master` and review the CI results before merging.

Do not force-push shared history as a release repair. Review unexpected local
changes before updating a checkout; preserve them instead of automatically
stashing, resetting or cleaning them.

## Developer Validation

Plain `px4xplane` automates setup using official PX4 `main`, with no pending
patches. `--validation` selects only open TECS #27670. Existing repo, branch
and path overrides remain available. See
[launcher validation](guided-launcher.md#developer-validation) for commands,
`--no-sync` behavior and feature-branch helper updates.

Use separate per-airframe runtime state. Source updates must not silently
delete saved tuning. `--reset-params` is an explicit backup/reset of the
selected runtime only. Record source revision, plugin version, aircraft,
active mapping and saved parameters when comparing results.

Model-specific work is covered in the
[compact QuadTailsitter developer page](compact-qtailsitter.md). Source-only
owner tools are not release assets. Keep private inputs and credentials out of
public branches and artifacts.

## Local Checks

Follow [BUILD.md](BUILD.md) for platform dependencies and packaging details.
For a CMake build with offline tests enabled:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DPX4XPLANE_BUILD_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Before any flight check, match the X-Plane aircraft, plugin `config_name` and
PX4 `SYS_AUTOSTART`. Offline tests and a successful build are not evidence of
a new live flight or full-envelope acceptance. Use the matching
[flight-test card](index.md#flight-test-cards) for runtime checks.

## CI and Packaging

| Trigger | Result |
| --- | --- |
| Pull request to `master` | Cross-platform build and offline tests |
| Manual dispatch of `build.yml` | On-demand CI build |
| Push of a new `v*.*.*` tag | Release workflow and versioned platform ZIPs |

Regular pushes to `master` do not create releases. Workflow definitions:
[build.yml](../.github/workflows/build.yml) and
[release.yml](../.github/workflows/release.yml).

Check the complete staged plugin folder, including `64/{platform}.xpl`,
`64/config.ini`, public `px4_airframes` references and user docs. Confirm
experimental profiles, owner runners and private artifacts are not included
as normal release assets. Use
[GitHub Actions logs](https://github.com/alireza787b/px4xplane/actions) to
investigate a failed build or packaging step.

## Versioning and Releases

Use semantic versioning: patch for compatible fixes, minor for compatible
features, major for breaking changes. Until a new release is approved, keep
`4.2.1` current and put changes under `Unreleased`.

For a separately approved future release:

1. Choose a new, unused version and synchronize `include/VersionInfo.h`,
   `CMakeLists.txt`, `Makefile.linux`, `Makefile.macos` and the config editor's
   displayed version/build.
2. Update current-package references in README, the docs index, config and
   aircraft/test-card docs. Move reviewed `Unreleased` entries to the new
   dated changelog section.
3. Verify offline tests, all platform packages and the relevant X-Plane checks.
4. Create a new annotated version tag only after approval, then monitor the
   release workflow and inspect the downloadable assets.

Published versions and tags are immutable. Do not reuse a tag, force-update it
or delete a published release to replace its contents. Document a published
mistake and prepare a new reviewed patch version instead.

See the [documentation index](index.md) for other guides and
[Troubleshooting](index.md#troubleshooting) for operator issues.
