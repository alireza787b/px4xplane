# Report v20 - v3.4.8 Release Build Recovery And Test Kit

Date: 2026-05-07

Scope: finish the config safety/editor release package and recover the GitHub
Windows release build after CI exposed an MSVC-only portability issue.

## What Happened

`v3.4.7` was tagged after the config safety/editor slices. GitHub created the
release and uploaded Linux/macOS assets, but the Windows MSVC release job failed.

Root cause:

- MSVC saw Windows `max` macro expansion inside
  `std::numeric_limits<uint64_t>::max()` in `src/MAVLinkManager.cpp`.
- Local MinGW Windows builds passed, so this only surfaced in the GitHub
  Windows/MSVC release job.

Fix:

- Added `NOMINMAX` to Windows compile definitions.
- Changed the one vulnerable call to `(std::numeric_limits<uint64_t>::max)()`.
- Bumped the project to `v3.4.8` instead of rewriting the already-published
  `v3.4.7` tag.

## JSON And INI Status

`config.ini` remains the runtime config. The plugin still reads
`px4xplane/64/config.ini`.

`config_schema.json` remains metadata only. It supports validation, docs, and
the local editor. It does not replace INI in `v3.4.8`.

The current editor flow remains:

1. Open `px4xplane/docs/config-editor.html`.
2. Import `px4xplane/64/config.ini`.
3. Optionally import `px4xplane/64/config_schema.json`.
4. Review/edit/add/remove airframes or channel mappings.
5. Export a clean `config.ini`.
6. Replace runtime INI deliberately.
7. Use `Advanced > Reload and Validate Config` before flight.

## Local Test Kit

Created:

- `/home/alireza/px4xplane-v3.4.8-next-test-kit-20260507.zip`

Contents:

- `px4xplane-windows-v3.4.8-test016-20260507.zip`
- `XPlaneTruthCapture-Windows-v0.1.7.zip`
- `5020_xplane_alia250_v3.4.8_test016`
- `README_NEXT_TEST.md`

Important: the Alia PX4 airframe file is frozen at the successful baseline.
`v3.4.8` does not include new Alia tuning or bridge sensor-contract changes.

## PX4 Fork State

Checked `/home/alireza/PX4-Autopilot-Me`:

- branch: `px4xplane-sitl`
- worktree: clean
- Alia airframe hash matches the px4xplane packaged/source copy

No PX4 fork commit was needed in this slice.

## Verification

Passed:

- `git diff --check`
- `python3 -m unittest tests.test_validate_config tests.test_replay_truth_capture`
- `node tests/test_config_editor.js`
- `node --check docs/assets/config-editor.js`
- `python3 tools/validate_config.py config/config.ini`
- `cmake -S . -B build-offline-tests -DCMAKE_BUILD_TYPE=Debug -DPX4XPLANE_BUILD_TESTS=ON`
- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`

Existing third-party MAVLink packed-member warnings and existing UI
unused-parameter warnings remain. No new runtime warning policy was introduced.

## Next Test Recommendation

Use the v3.4.8 local test kit for the next lab run. Keep Alia params frozen, run
the same successful Alia scenario, and collect:

- PX4 ULog
- X-Plane `Log.txt`
- PX4 CLI log
- XPlaneTruthCapture run folder

If the test passes, the next implementation slice should focus on safer config
UX refinements and replay analysis, not Alia tuning changes yet.
