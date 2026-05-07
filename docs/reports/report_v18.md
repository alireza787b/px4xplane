# Report v18 - Static Config Editor Prototype

Date: 2026-05-07

Scope: add the first schema-backed config editor prototype for px4xplane. This
is a tooling/UI slice only. It does not change bridge runtime behavior, Alia PX4
parameters, sensor signs/units, quaternion handling, actuator output logic, or
prop-brake policy.

## What Is Happening With JSON And INI

`config.ini` is still the runtime configuration file. The plugin still reads
`64/config.ini`, and exported packages still include that file.

`config_schema.json` is metadata, not a runtime replacement. It tells tools and
the future UI:

- which fields exist
- field type: string, bool, int, float, actuator mapping
- safe numeric ranges
- whether a field is live-reloadable or setup-time only
- supported channel mapping types: `float`, `floatArray`

Recommended migration policy:

1. Keep `config.ini` as the runtime format for compatibility.
2. Use JSON schema for validation, docs, and editor UX.
3. Let the editor export clean `config.ini`.
4. Only consider JSON-as-runtime later if the plugin can read both formats and
   generate/round-trip INI safely.

So no, JSON is not replacing INI now. It is reducing confusion around INI by
making the valid fields and policies explicit.

## Example

Runtime `config.ini` stays simple:

```ini
config_name = ExampleVTOL
diagnostic_log_enabled = true
fps_warning_threshold = 50

[ExampleVTOL]
autoPropBrakes = 0, 1
channel0 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]
channel1 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [1], [-1 1]
channel4 = sim/flightmodel/controls/wing1l_ail1def, float, 0, [-20 20]
```

Schema metadata explains how tools should treat those fields:

```json
{
  "fps_warning_threshold": {
    "type": "int",
    "min": 20,
    "max": 120,
    "reload_policy": "live_reload"
  },
  "channelN": {
    "type": "actuator_mapping",
    "min_channel": 0,
    "max_channel": 15,
    "reload_policy": "reconnect_before_flight"
  }
}
```

The practical result: normal users edit fields through a clear UI and export
INI; advanced users still have full dataref/channel flexibility.

## What Changed

- Added `docs/config-editor.html`.
- Added `docs/assets/config-editor.js`.
- The editor can:
  - import `config.ini`
  - optionally import `config_schema.json`
  - list airframes
  - set active airframe
  - add, rename, and remove airframes
  - edit schema-backed global fields
  - edit `autoPropBrakes`
  - add, edit, and remove channel mappings
  - validate active config and mappings
  - export a clean `config.ini`
  - show an export preview
- Added `tests/test_config_editor.js` for parser/serializer/validator coverage.
- Wired the Node-based editor smoke test into optional CTest when `node` is
  available.
- CMake now packages the editor under `px4xplane/docs/`:
  - `docs/config-editor.html`
  - `docs/assets/config-editor.js`
- README and docs index now link to the editor.

## UX Decision

The editor is intentionally a static local tool. It does not write directly into
X-Plane folders, PX4 folders, or a running simulator. That avoids silent config
changes during flight and avoids platform-specific file permission problems.

The intended workflow is:

1. Open `docs/config-editor.html`.
2. Import the current `64/config.ini`.
3. Review/edit fields and channel mappings.
4. Resolve validation errors.
5. Export `config.ini`.
6. Replace the runtime config deliberately.
7. Use px4xplane `Advanced > Reload and Validate Config`, or reconnect PX4 when
   the field policy says reconnect before flight.

## What Did Not Change

- No automatic file save into X-Plane.
- No plugin menu item opens the editor yet.
- No JSON runtime parser in the plugin.
- No migration away from `config.ini`.
- No aircraft tuning changes.
- No XPlaneTruthCapture changes.

## Verification

Passed:

- `node tests/test_config_editor.js`
- `node --check docs/assets/config-editor.js`
- `python3 -m unittest tests.test_validate_config tests.test_replay_truth_capture`
- `python3 tools/validate_config.py config/config.ini`
- `python3 -m py_compile tools/validate_config.py tools/replay_truth_capture.py tools/analyze_ulog_estimator.py`
- `cmake -S . -B build-offline-tests -DCMAKE_BUILD_TYPE=Debug -DPX4XPLANE_BUILD_TESTS=ON`
- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`
- `git diff --check`

The Linux and MinGW Windows plugin build outputs contain the editor files under
`px4xplane/docs/`.

## Next Slice

Recommended next slice:

1. Add an X-Plane Advanced menu item that points users to the packaged editor
   location without trying unsafe automatic browser launching yet.
2. Add editor support for importing/exporting airframe sections as standalone
   JSON snippets for sharing, while still exporting runtime `config.ini`.
3. Keep Alia parameters frozen until the next full retest returns complete
   evidence.
