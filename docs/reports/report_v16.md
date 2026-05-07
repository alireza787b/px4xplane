# Report v16 - Runtime Config Validation UI Slice

Date: 2026-05-07

Scope: second approved px4xplane modernization slice after the Alia retest
triage. This slice improves runtime observability and menu workflow only. It
does not change Alia PX4 parameters, sensor signs, MAVLink units, quaternion
handling, or the prop-brake policy.

## What Changed

- Added an in-process `ConfigValidationSummary` model to `ConfigManager`.
- `ConfigManager::loadConfiguration()` now validates the active X-Plane-side
  actuator mapping after parsing.
- Runtime validation checks:
  - missing `config.ini`
  - empty `config_name`
  - no parsed actuator channels
  - channels outside `0..15`
  - channels with no dataref mappings
  - empty dataref names
  - unsupported runtime data types
  - `floatArray` mappings without indices
  - negative array indices
  - non-finite or equal output ranges
  - configured datarefs missing from the currently loaded X-Plane aircraft
- Missing `config.ini` now leaves a visible validation error instead of a stale
  or blank status after reload.
- Added a top-level `Advanced` submenu:
  - current config validation headline
  - `Validate Config`
  - `Reload and Validate Config`
  - `Bridge Diagnostics: On/Off`
  - `Show Docs Location`
- Moved the diagnostics toggle out of the normal top-level menu and into the
  advanced menu.
- The in-plugin Connection and Mixing tabs now show config validation status.
  The Mixing tab shows the first three validation messages for quick triage.
- Added a stable docs entry point at `docs/index.md`.
- Hardened menu lifecycle cleanup:
  - child submenus are destroyed before menu reconstruction
  - plugin window and menus are destroyed during `XPluginStop`

## Design Decisions

- The normal menu stays small: airframe selection, data window, config reload,
  advanced tools, and connect/disconnect.
- Validation against `XPLMFindDataRef()` is intentionally aircraft-dependent.
  If the active config is `Alia250` but X-Plane has a different aircraft loaded,
  the validator should report missing active-aircraft datarefs. That catches the
  exact class of "wrong airframe/mapping loaded" problem seen during retest
  triage.
- `Show Docs Location` writes the docs URL to X-Plane `Log.txt` and the plugin
  status line instead of launching a browser. Browser/file launching remains a
  later platform-specific UX slice because it needs Windows/macOS/Linux handling
  and failure behavior.
- The future HTML/schema config editor was not started yet. This slice creates
  the runtime validation state that the editor should consume later.

## What Did Not Change

- Alia `5020_xplane_alia250` parameters remain frozen at the v3.4.6 baseline.
- No change was made to bridge sensor signs, units, GPS COG, HIL state fields,
  or quaternion conversion.
- No XPlaneTruthCapture changes were made in this px4xplane slice.
- No generalized prop-brake redesign was implemented.
- No release package was produced in this slice; this is a source/test
  checkpoint before the larger UI/config-editor architecture work.

## Verification

Passed:

- `python3 -m unittest tests.test_validate_config tests.test_replay_truth_capture`
- `python3 tools/validate_config.py config/config.ini`
- `python3 -m py_compile tools/validate_config.py tools/replay_truth_capture.py tools/analyze_ulog_estimator.py`
- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`
- `x86_64-w64-mingw32-objdump -p build-win-mingw/win/Release/px4xplane/64/win.xpl`

The Windows plugin imports remain limited to expected X-Plane/runtime/system
libraries: `XPLM_64.dll`, `KERNEL32.dll`, `msvcrt.dll`, `OPENGL32.dll`, and
`WS2_32.dll`. No MinGW `libgcc_s_seh-1.dll` or `libstdc++-6.dll` runtime import
appeared.

Existing warnings remain from MAVLink packed members, older unused callback
parameters, and one signed/unsigned comparison in existing config-list code.
They are not introduced by the validation model.

## Next Slice

Recommended next slice:

1. Add schema-backed config metadata for supported fields, reload policy, and
   user-facing descriptions.
2. Use that schema in both the offline validator and the future config editor.
3. Keep the editor simple for normal users and expose advanced mapping details
   only in the advanced path.
4. Keep Alia params frozen until the next real X-Plane retest returns full
   `Log.txt`, ULog, and truth-capture artifacts.
