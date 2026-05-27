# Report v64 - Config Editor and Camera Workflow Closure

Package target: `v3.4.52`

## Scope

This slice closes the PM feedback before moving to Cessna/TB2:

- make the config-driven camera workflow available to all packaged airframes
- remove JSON-vs-INI confusion from the runtime package layout
- make the config editor easier to use and self-validating without manual schema import
- make X-Plane menu links actually open docs/editor/repo when the OS allows it
- eliminate false validation warnings from the editor fallback schema

No PX4 airframe parameters or QuadTailsitter flight-control tuning changed in
this slice.

## Decisions

Runtime source of truth remains `px4xplane/64/config.ini`.

`config/config_schema.json` is not a second runtime config and is not read by
the plugin. It is metadata for validation, field descriptions, reload policies,
and the local editor. To reduce operator confusion, packaged builds no longer
place `config_schema.json` next to the plugin binary. The package now puts it at:

```text
px4xplane/docs/config_schema.json
```

The editor also embeds the same schema-shaped metadata as a fallback so users do
not get false warnings when browser local-file security blocks automatic reads.
The source-tree schema remains the validator source for CLI checks.

## Evidence and References

- X-Plane SDK menus can trigger callbacks or commands. We kept command-backed
  camera views because commands are bindable in X-Plane keyboard settings:
  https://developer.x-plane.com/sdk/XPLMAppendMenuItemWithCommand/
- X-Plane plugin menus are sandboxed to the plugin and can be rebuilt safely
  after config reloads:
  https://developer.x-plane.com/sdk/XPLMMenus/
- Browser `file://` reads are not reliable for `fetch()` because modern browsers
  may treat local files as opaque origins. The editor therefore tries auto-load,
  then falls back to embedded schema/manual file load:
  https://developer.mozilla.org/en-US/docs/Web/HTTP/Guides/CORS/Errors/CORSRequestNotHttp

## Changes

Camera workflow:

- Added `cameraViews` to Ehang 184, Cessna 172, Alia 250, and TB2.
- Kept QuadTailsitter's accepted nose/FPV, belly/down, and chase views.
- Rebuilt the camera menu after airframe selection and released any active
  plugin camera before switching airframes. This prevents stale camera labels or
  stale camera indices after changing active aircraft.

Menu/link workflow:

- Replaced `Show Config Editor Location` with `Open Config Editor`.
- Replaced `Show Docs Location` with `Open Documentation`.
- Added `Open GitHub Repository`.
- Implemented OS-native external opening:
  - Windows: `ShellExecuteA`
  - macOS: detached `open`
  - Linux: detached `xdg-open`
- Kept the existing safe fallback: if opening fails, the exact path/URL is still
  written to X-Plane `Log.txt` and shown in the status HUD.

Config editor:

- Added dark mode with system preference support and a manual toggle.
- Added automatic package loading for `../64/config.ini` and
  `docs/config_schema.json` when the browser allows local reads.
- Added clear runtime/source status text: `64/config.ini` is the only runtime
  source; JSON is metadata.
- Added a structured camera editor with add/remove rows and numeric fields
  instead of a raw pipe-delimited text box.
- Added inline help buttons, docs links, reload-policy badges, footer links, and
  version/build metadata.
- Expanded the embedded fallback schema so current release configs no longer
  produce false `unknown global key` warnings when the user does not manually
  import the JSON schema.
- Added exact zoom validation in both JS and Python validators to match runtime
  camera parsing (`0 < zoom <= 4`).

Packaging:

- Stopped copying `config_schema.json` into `px4xplane/64/`.
- Copied schema metadata into `px4xplane/docs/config_schema.json`.
- Removed stale `64/config_schema.json` during package refresh so old local
  build folders do not carry the obsolete layout.

## Review Notes

An independent code review found three concrete risks in the old slice:

- editor warnings were caused by the incomplete fallback schema
- the camera menu could become stale after airframe switching
- browser launching should avoid shell-quoted `system()` calls

All three are addressed in this package.

## Verification

Run:

```bash
python3 tools/validate_config.py config/config.ini
node --check docs/assets/config-editor.js
node tests/test_config_editor.js
ctest --test-dir build --output-on-failure
cmake --build build --target px4xplane
cmake --build build-win-mingw --target px4xplane
```

The editor test now also compares the embedded fallback schema keys with
`config/config_schema.json`, so release validation catches future drift before
false warnings return.

Manual checks for the lab:

1. Install the new package.
2. In X-Plane, open `PX4 X-Plane > Advanced > Open Config Editor`.
3. Confirm the editor opens in the default browser.
4. Confirm no false unknown-key warnings for the packaged config.
5. Switch between Alia, Ehang, Cessna, TB2, and QuadTailsitter and confirm
   `PX4 X-Plane > Camera Views` updates to the active airframe.

If a browser blocks local auto-loading, use the file picker for
`px4xplane/64/config.ini`. This is browser security behavior, not a plugin
runtime issue.
