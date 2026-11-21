# Developer Guide

Current published package: `v4.2.1`. Source changes belong under `Unreleased`
until a separately approved version and release are prepared.

Normal setup uses native official PX4 as described in the
[Quick Start](../README.md#quick-start). The optional
[beta/developer launcher](guided-launcher.md) provides guided project setup
and source comparisons.

## Development Workflow

1. Work on a feature branch and keep changes scoped to the task.
2. Build and test locally using the [Build Guide](BUILD.md).
3. Add focused tests and an `Unreleased` changelog entry for behavioral changes.
4. Open a pull request against `master` and review the CI results before merging.

Do not force-push shared history as a release repair. Review unexpected local
changes before updating a checkout; preserve them instead of automatically
stashing, resetting or cleaning them.

## Developer Validation

Plain `px4xplane` selects `--maintained`: the project repository
`https://github.com/alireza787b/PX4-Autopilot-Me.git`, branch
`px4xplane-airframes`. The integration baseline is upstream `0e0f3ce8a2` plus
the project's five airframe files and a separate calibration-slot fix. The
calibration utility now skips nonexistent/unreadable slots instead of treating
them as free; it does not change estimator or controller tuning. Keep updates to that branch reviewable
against their recorded upstream base; do not add a controller patch stack as
an implicit setup step.

The beta bootstrap updates its installed launcher pair before PX4 setup and
build, so a failed checkout update does not leave an old command installed.
Stashing needs explicit approval; retain recovery identities and never
automatically apply a stash to a different revision. Check nested repository
work independently. A consented rebuild must target only SITL artifacts, not
PX4's broad clean/distclean or ignored user files.

`--official` selects bare upstream `PX4/PX4-Autopilot` `main`, without project
airframe overlays or pending patches. Upstream's available targets and default
values can differ. Project reference changes do not establish upstream merge
status. See [source modes](guided-launcher.md#source-modes) for selection and
[checkout safety](guided-launcher.md#existing-checkouts-and-sync) for custom
repo/branch/path overrides and no-sync runs.

Use separate per-airframe runtime state. Source updates must not silently
delete saved tuning. `--reset-params` is an explicit backup/reset of the
selected runtime only. Record source revision, plugin version, aircraft,
active mapping and saved parameters when comparing results.

The [QuadTailsitter card](QUADTAILSITTER_XPLANE12_TEST.md) is the single model
and profile guide. Keep the names, mapping and target unchanged; do not add
another profile runner or public aircraft variant. Aircraft files are supplied
separately and do not belong in the public source or release assets.

## Airframe Reference Contract

Maintain `config/px4_params/` as reference copies of the maintained branch's
`ROMFS/px4fmu_common/init.d-posix/airframes/` files for these IDs:
`5001`, `5002`, `5010`, `5020` and `5021`. Compare the files at an exact source
revision, not just target names or headings. Update schema/parameter checks
when upstream retires a parameter; do not keep an obsolete second profile.

QuadTailsitter has one canonical 209-value reference. The current failure
detector uses `FD_TILT_ERROR` and `FD_TILT_ERROR_T` in place of the retired
four roll/pitch failure-detector parameters. Preserve the other frozen profile values
unless a separate, evidence-backed tuning change is approved.

PX4 runs the airframe script embedded in its selected build, after inherited
vehicle defaults. `param set-default` does not erase saved overrides. The
plugin's packaged `px4_airframes` copies are not executed; editing or replacing
them alone does not alter SITL. Rebuild after source changes and inspect the
effective parameters, not just the reference file. See the
[reference notes](../config/px4_params/README.md).

## Verification Limits

A profile's canonical status is an integration decision, not new flight
evidence. Builds, parameter-contract checks and network/startup tests must be
reported separately from X-Plane flight results on the exact source revision.

For QuadTailsitter, retain review of `FW_R_TC=1.5` against the upstream metadata
maximum of `1.0`; silently clamping it changes the control response. Validate
the current failure-detector conversion, wing/clearance metadata, actual model
mass/CG and speed/stall limits. Prior tests exposed fixed-wing oscillation and
back-transition climb; they do not establish the envelope on this upstream
base. Follow the flight card for controlled checks. SITL gains are not hardware
firmware or hardware acceptance evidence.

Other accepted profiles also contain values outside PX4's parameter-editor
metadata bounds, including Alia transition times and speed. Some predate this
integration. Do not silently clamp them or widen shared bounds just to make a
range check pass; review the model envelope and the applicable controller code
before changing them.

PX4 output, effective parameters and ULogs are normal diagnostic inputs.
TruthCapture is optional for investigations needing simulator ground truth,
not an installation, launch or routine-flight requirement. Share sanitized
metrics; keep privately supplied aircraft and aircraft-bearing raw evidence
out of public reports and artifacts.

## Local Checks

Follow [BUILD.md](BUILD.md) for platform dependencies and packaging details.
For a CMake build with offline tests enabled:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DPX4XPLANE_BUILD_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

From the px4xplane source checkout, check the five reference files against a
local checkout of the maintained PX4 branch before publishing an airframe update:

```bash
python3 tools/check_airframe_references.py --px4-path ~/PX4-Autopilot
```

This source-only maintenance tool is not bundled with the plugin. Its default
reference directory is `config/px4_params` in the source checkout.

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
[build.yml](https://github.com/alireza787b/px4xplane/blob/master/.github/workflows/build.yml) and
[release.yml](https://github.com/alireza787b/px4xplane/blob/master/.github/workflows/release.yml).

Check the complete staged plugin folder, including `64/{platform}.xpl`,
`64/config.ini`, the five canonical `px4_airframes` references and user docs.
Confirm that no duplicate model/profile workflow, aircraft file or owner-only
artifact is included. Use
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
