# px4xplane Config Schema

`config/config_schema.json` is the machine-readable metadata for the current
`config.ini` format. It is intentionally small: it documents existing fields,
their types, safe ranges, and reload policy without changing the runtime parser
or aircraft behavior.

## Current Format

`config.ini` remains the runtime config file. The schema currently covers:

- global config fields before the first airframe section
- supported actuator mapping types: `float` and `floatArray`
- channel range: `channel0` through `channel15`
- X-Plane motor index range for `autoPropBrakes`: `0` through `7`
- prop-brake policy fields: apply/release thresholds, dwell, optional
  true-airspeed gate, mode, and experimental failure-dataref use
- optional airframe camera presets in `cameraViews`
- config reload policy for user-facing tools and docs

Run the validator before packaging:

```bash
python3 tools/validate_config.py config/config.ini
```

List schema-backed fields and reload policy:

```bash
python3 tools/validate_config.py --list-fields
```

## Reload Policy

`live_reload` fields are safe to update with `Reload Config` for normal
diagnostic use:

- compact diagnostic logging
- diagnostic interval
- connection HUD
- FPS warning settings
- airframe camera preset definitions

`reconnect_before_flight` fields must be treated as setup-time fields:

- active airframe name
- actuator channel mappings
- prop-brake motor list
- prop-brake policy fields
- accelerometer calibration/offset fields
- MAVLink target message rates

Some of these values are technically reparsed by `Reload Config`, but changing
them while armed or mid-flight is not supported. Disconnect PX4 SITL, reload,
validate, reconnect, and then fly.

`development_only` fields are high-volume or contract-bypassing debug controls.
Keep them disabled for normal release packages and demo runs.

`px4_sitl_restart` is reserved for PX4 airframe parameters. PX4 parameters are
not changed by `config.ini`; change the PX4 airframe file or saved parameters
and restart/reset PX4 SITL.

## Editor Direction

The future config editor should consume this schema first. Do not build a raw
INI editor as the primary UX. The normal path should show a small set of fields
with validation, while advanced users can edit channel mappings, ranges, and
datarefs with inline warnings.

The static editor lives at `docs/config-editor.html`. Runtime behavior still
comes from `64/config.ini`; the schema is loaded automatically from
`docs/config_schema.json` when the browser allows local reads and is also
embedded in the editor as a fallback. The editor validates the editable model
and exports a clean `config.ini`. It does not write into X-Plane or PX4
directly.

The plugin does not read `config_schema.json`. CMake packages it under
`px4xplane/docs/` so it is visibly documentation/tooling metadata rather than a
second runtime config next to the plugin binary.
