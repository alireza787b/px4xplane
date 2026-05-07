# Report v15 - Config Safety Slice

Date: 2026-05-07

Scope: complete the first approved offline implementation slice for px4xplane
after the Alia retest triage. This is a behavior-preserving safety slice: no
Alia PX4 parameter tuning, sensor sign/unit change, quaternion change, or prop
brake policy rewrite is included.

## What Changed

- Config now loads at plugin startup before menu/UI state is created.
- `ConfigManager::loadConfiguration()` clears previous actuator mappings before
  parsing, so switching from one airframe to another cannot leave stale channels
  in memory.
- Actuator scaling moved into a small tested helper:
  - non-finite HIL actuator inputs fall back to neutral
  - channel commands are clamped to the expected PX4 `[-1, 1]` range
  - output ranges are checked for finite non-equal endpoints
  - reversed min/max ranges are handled by the clamp helper
- Runtime actuator writes now skip invalid ranges and log the affected channel.
- Configured actuator datarefs are zeroed once if `HIL_ACTUATOR_CONTROLS`
  becomes stale or has not arrived yet after connection.
- Inbound MAVLink receive now drains up to 16 chunks per X-Plane frame instead
  of one small read per frame. This reduces backlog risk on low-FPS or bursty
  systems while keeping a bounded per-frame budget.
- Prop brake scanning now runs once per actuator override pass instead of once
  per configured channel. The existing configurable brake concept is preserved;
  the full typed prop-brake policy remains a later slice.
- Missing float-array datarefs now return an empty vector instead of calling
  `XPLMGetDatavf()` on a null dataref.
- Added `tools/validate_config.py` for offline `config.ini` validation:
  - active `config_name` section
  - channel key range
  - mapping token format
  - supported runtime types: `float`, `floatArray`
  - array index syntax
  - finite, non-equal output ranges
  - `autoPropBrakes` motor index range
- Added Python tests for config validator failure modes and wired Python offline
  checks into optional CTest.
- Updated custom-airframe docs and in-plugin Mixing tab help so reload/reconnect
  behavior is no longer misleading.
- Added `build-*/` to `.gitignore` for local out-of-tree build directories.

## What Did Not Change

- Alia `5020_xplane_alia250` parameters remain frozen at the v3.4.6 baseline.
- The v3.4.3/v3.4.6 sensor contract remains unchanged.
- No changes were made to XPlaneTruthCapture in this slice.
- The prop-brake failure-dataref mechanism was not redesigned yet.
- The future HTML/JS config editor was not started yet; this slice creates the
  validator/test foundation it will use.

## Verification

Passed:

- `python3 -m unittest tests.test_validate_config tests.test_replay_truth_capture`
- `python3 tools/validate_config.py config/config.ini`
- `python3 -m py_compile tools/validate_config.py tools/replay_truth_capture.py tools/analyze_ulog_estimator.py`
- `cmake -S . -B build-offline-tests -DCMAKE_BUILD_TYPE=Debug -DPX4XPLANE_BUILD_TESTS=ON`
- `cmake --build build-offline-tests --target actuator_safety_test`
- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`
- `x86_64-w64-mingw32-objdump -p build-win-mingw/win/Release/px4xplane/64/win.xpl`

The MinGW Windows plugin imports remain limited to expected runtime/system
libraries: `XPLM_64.dll`, `KERNEL32.dll`, `msvcrt.dll`, `OPENGL32.dll`, and
`WS2_32.dll`. No `libgcc_s_seh-1.dll` or `libstdc++-6.dll` dependency appeared.

Build warnings remain from existing MAVLink packed-member headers, Eigen, and
older unused UI parameters. They are not introduced by this slice.

## Review Notes

This slice removes several ways a wrong or stale X-Plane-side mapping could look
like a bad aircraft tune:

- stale channels cannot survive reload
- invalid output ranges cannot silently write nonsense
- NaN/Inf actuator inputs cannot reach datarefs
- PX4 actuator stream loss cannot keep driving the last command indefinitely
- receive backlog risk is lower on slower machines

It still does not prove passenger-grade Alia tuning. That remains blocked on the
next controlled X-Plane run with full ULog, X-Plane `Log.txt`, and
XPlaneTruthCapture evidence.

## Next Slice

Recommended next slice:

1. Add runtime config validation/status presentation in the X-Plane UI.
2. Add a simple menu path for `Validate Config`, `Open Docs`, and later
   `Open Config Editor`.
3. Start the schema-first config editor only after the runtime validation model
   is stable.
4. Keep Alia params frozen until the v3.4.6 baseline retest returns complete
   artifacts.
