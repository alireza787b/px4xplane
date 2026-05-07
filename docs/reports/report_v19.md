# Report v19 - Config Editor Menu Link Slice

Date: 2026-05-07

Scope: connect the new packaged config editor to the in-sim workflow without
changing flight behavior.

## What Changed

- Added `Advanced > Show Config Editor Location` to the px4xplane menu.
- The menu action resolves the loaded plugin binary path, derives the plugin
  root, and writes this editor path to X-Plane `Log.txt`:
  - `px4xplane/docs/config-editor.html`
- The plugin status line also reports that the config editor path was written.

## Design Decision

The menu does not launch a browser yet. That is deliberate.

Opening files or URLs from an X-Plane plugin needs platform-specific handling
for Windows, macOS, and Linux, plus clear failure behavior. This slice gives
users a reliable path in `Log.txt` first. A later UX slice can add safe
platform-specific launching after it is tested on all target platforms.

## What Did Not Change

- No runtime config format change.
- No JSON parser in the plugin.
- No Alia tuning change.
- No sensor-contract change.
- No XPlaneTruthCapture change.

## Verification

Passed:

- `ctest --test-dir build-offline-tests --output-on-failure`
- `cmake --build build --config Release --parallel 2`
- `cmake --build build-win-mingw --config Release --parallel 2`
- `git diff --check`

Existing MAVLink packed-member warnings remain unchanged.

## Next Slice

Recommended next slice:

1. Package a new Windows test bundle from the current `master`, including the
   editor and schema.
2. Keep Alia params frozen for the next lab run.
3. Add safe platform-specific editor/docs launching only after this path-based
   workflow is confirmed inside X-Plane.
