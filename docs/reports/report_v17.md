# Report v17 - Config Schema Metadata Slice

Date: 2026-05-07

Scope: add schema-backed config metadata for px4xplane without changing bridge
flight behavior. This is groundwork for safer validation, documentation, and a
future config editor. Alia PX4 parameters, sensor signs/units, quaternion
handling, actuator output behavior, and prop-brake policy are unchanged.

## What Changed

- Added `config/config_schema.json` as the metadata source for the current
  `config.ini` format.
- The schema now records:
  - supported runtime actuator mapping types: `float`, `floatArray`
  - global config field types
  - defaults where they are known
  - safe min/max ranges for bounded numeric fields
  - field groups
  - reload policy
  - channel and `autoPropBrakes` ranges
- `tools/validate_config.py` now loads the schema and validates global fields:
  - unknown global keys become warnings
  - invalid booleans become errors
  - invalid integer/float values become errors
  - out-of-range global values become errors
- Added `--schema` to test alternate schema files.
- Added `--list-fields` to print schema-backed fields and reload policies.
- CMake now copies `config_schema.json` into the packaged plugin folder next to
  `config.ini`.
- Added developer docs at `docs/developer/config-schema.md`.
- Updated custom-airframe docs with live-reload vs reconnect-before-flight
  guidance.
- Linked the config schema docs from `docs/index.md`.
- Added validator tests for schema-backed global field checks.

## Reload Policy Decision

The schema uses four policy labels:

- `live_reload`: safe to reload for normal diagnostic/UI use.
- `reconnect_before_flight`: setup-time fields. Reload may parse them, but users
  should disconnect/reconnect PX4 before flying.
- `development_only`: high-volume or contract-bypassing debug controls.
- `px4_sitl_restart`: PX4 airframe parameters, not `config.ini` runtime fields.

Current safe live-reload group:

- compact diagnostic logging
- diagnostic interval
- connection HUD
- FPS warning settings

Current reconnect-before-flight group:

- active config name
- actuator mappings
- prop-brake motor list
- accelerometer calibration/offset fields
- MAVLink target rates

This keeps the normal workflow simple and avoids pretending that every config
field is safe to edit while armed.

## What Did Not Change

- The plugin runtime parser still reads `config.ini`; it does not read the JSON
  schema yet.
- No HTML/JS config editor was started.
- No Alia tuning or PX4 fork parameter change was made.
- No XPlaneTruthCapture change was made in this slice.
- No release tag/package was produced for this source checkpoint.

## Verification

Passed:

- `jq . config/config_schema.json`
- `python3 -m unittest tests.test_validate_config tests.test_replay_truth_capture`
- `python3 tools/validate_config.py config/config.ini`
- `python3 tools/validate_config.py --list-fields`
- `python3 -m py_compile tools/validate_config.py tools/replay_truth_capture.py tools/analyze_ulog_estimator.py`
- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`
- `git diff --check`

Both Linux and MinGW Windows plugin builds copied `config_schema.json` into the
plugin `64/` directory.

## Next Slice

Recommended next slice:

1. Add a small generated schema summary artifact for docs/editor use, or expose
   the schema in the plugin UI status view.
2. Then build the first config-editor prototype against the schema, starting as
   a static local HTML/JS tool with validation and no in-sim file writes.
3. Keep Alia parameters frozen until the next full X-Plane retest returns ULog,
   X-Plane `Log.txt`, and XPlaneTruthCapture evidence.
