# Defining Custom Airframes in PX4-XPlane

This guide explains how to create or modify a px4xplane airframe configuration using the browser config editor and the runtime `config.ini` file.

## Configuration Model

px4xplane uses one runtime source of truth:

```text
px4xplane/64/config.ini
```

`config_schema.json` is validation metadata for tools and the editor. It is not read by the plugin at runtime and should not be loaded as the active aircraft config.

The editor-first workflow is recommended:

1. Open the config editor.
2. Load `px4xplane/64/config.ini` if auto-load is blocked.
3. Add or edit the airframe section.
4. Validate warnings and ranges.
5. Export the edited `config.ini`.
6. Replace `px4xplane/64/config.ini`.
7. Reload config in X-Plane or reconnect PX4 before flight, depending on the changed fields.

![px4xplane config editor](assets/config-editor-ui.png)

## Open the Config Editor

From X-Plane:

```text
Plugins > PX4 X-Plane > Advanced > Open Config Editor
```

From a release package:

```text
px4xplane/docs/config-editor.html
```

From the source tree:

```text
docs/config-editor.html
```

If the browser blocks local-file auto-load, use the main file picker to choose:

```text
px4xplane/64/config.ini
```

Only use the advanced schema JSON picker when you intentionally want to load custom schema metadata.

## What Can Be Integrated

The packaged examples cover fixed-wing, multicopter, and VTOL aircraft. A new model can be integrated when all of these are true:

- X-Plane can model the vehicle or aircraft behavior.
- The required X-Plane controls are exposed as writable datarefs.
- PX4 has a matching SITL-capable control path or airframe.
- The px4xplane `config.ini` maps PX4 actuator outputs to those writable datarefs.

Aircraft are the primary tested path today. Rovers, boats, helicopters, or other X-Plane vehicle models are possible in principle when the PX4 control path and dataref mapping exist, but they should be treated as custom integrations until validated.

Useful X-Plane references:

- [Plane Maker manual](https://developer.x-plane.com/manuals/planemaker/index.html)
- [X-Plane datarefs reference](https://developer.x-plane.com/datarefs/)
- [X-Plane developer documentation](https://developer.x-plane.com/)

## Build the X-Plane Model

Choose one path:

- Start from an existing X-Plane aircraft that is close to your target.
- Build a new model in Plane Maker.
- For plugin-controlled custom systems, expose writable datarefs that px4xplane can command.

In Plane Maker, confirm basic geometry, mass, CG, landing gear, engines, control surfaces, flaps, and steering before PX4 is connected. A poor X-Plane model cannot be fixed only with PX4 gains.

## Choose or Create the PX4 Airframe

Use the closest PX4 SITL airframe as a starting point:

- fixed-wing: Cessna 172 or TB2 style targets
- multicopter: Ehang 184 style targets
- standard VTOL: Alia 250 style targets
- tailsitter VTOL: QuadTailsitter style targets

For a new upstream PX4 target, create or update the PX4 airframe file, set `SYS_AUTOSTART`, configure control allocation, and tune parameters from ULog evidence. Use QGroundControl actuator setup to verify which PX4 actuator function appears on each output channel.

## Edit the px4xplane Mapping

Each airframe section in `config.ini` maps PX4 actuator channels to X-Plane datarefs.

Example:

```ini
config_name = Cessna172

[Cessna172]
aircraftMatch = Cessna,172
cameraViews = Forward|2.8|0.0|0.7|0.0|0.0|0.0|0.90; Down|0.4|0.0|-0.55|-90.0|0.0|0.0|0.85; Chase|-13.0|0.0|3.2|-8.0|0.0|0.0|0.70
actuatorSmoothingTimeConstantSec = 0.04
actuatorSmoothingChannels = 0, 1, 2, 4

channel0 = sim/flightmodel/controls/wing2l_ail1def, float, 0, [-20 +20]
channel1 = sim/flightmodel/controls/wing2r_ail1def, float, 0, [-20 +20]
channel2 = sim/flightmodel/controls/hstab1_elv1def, float, 0, [-20 +20]
channel3 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]
```

Channel format:

```text
channelN = dataref, type, index, [min max]
```

Multiple X-Plane datarefs can be driven from one PX4 channel by separating mappings with `|`.

Supported runtime types:

- `float`: scalar writable dataref, usually control surfaces or steering
- `floatArray`: array writable dataref, usually engine throttle arrays

PX4 actuator outputs are normalized floating-point setpoints, so px4xplane currently writes scalar floats and float-array elements. For integer-only aircraft systems, use aircraft-local plugin logic or add a typed schema/runtime extension before relying on it.

## Important Fields

`config_name`
: Active airframe section name. This must match one section exactly.

`aircraftMatch`
: Optional warning tokens checked against the loaded X-Plane aircraft file/path. This is a setup guard, not an automatic selector.

`airspeedSource`
: Simulated pitot source. `xplane_indicated` is normal for conventional aircraft. `body_axis` projects air-relative velocity onto `pitotAxisBody`, useful for tailsitters or custom pitot placement.

`cameraViews`
: Plugin-owned camera presets available from `PX4 X-Plane > Camera Views` and X-Plane command bindings such as `px4xplane/camera/view_1`.

`actuatorSmoothingTimeConstantSec`
: Optional first-order smoothing between PX4 HIL actuator packets and X-Plane frames. Use small values such as `0.02` to `0.05` only for visibly stepped fixed-wing surfaces. Do not use smoothing to hide unstable tuning.

`actuatorSmoothingChannels`
: Optional allow-list so only specific PX4 output channels are smoothed.

`autoPropBrakes`
: X-Plane engine indices that can be braked or feathered when commanded low. Use only when validated for the airframe.

## Reload Policy

Safe live-reload fields are limited to compact diagnostics, HUD visibility, FPS warning settings, and camera definitions.

Treat these as setup-time fields:

- active airframe name
- actuator channel mappings
- actuator smoothing
- aircraft match tokens
- prop-brake configuration
- accelerometer calibration/offset fields
- MAVLink target rates

For setup-time fields, disconnect PX4 SITL, save the new `config.ini`, replace `px4xplane/64/config.ini`, reload config in X-Plane, reconnect PX4, and then fly. Do not change actuator mappings while armed.

## Validate

From the source tree:

```bash
python3 tools/validate_config.py config/config.ini
python3 tools/validate_config.py --list-fields
```

From X-Plane:

```text
Plugins > PX4 X-Plane > Advanced > Validate Config
```

Before judging flight behavior, confirm:

- X-Plane loaded the intended aircraft.
- `config_name` points to the matching section.
- The connection HUD has no aircraft/config mismatch warning.
- PX4 started the intended `SYS_AUTOSTART` target.
- QGroundControl actuator outputs match the expected channel order.

## Test in Steps

1. Check datarefs with the aircraft parked and PX4 disconnected.
2. Start PX4 SITL and confirm sensor health before arming.
3. Move actuators from QGroundControl and verify X-Plane surfaces, motors, steering, and flaps.
4. Run a short manual or mission test.
5. Compare PX4 ULog data with X-Plane truth logs before changing gains.
6. Update PX4 parameters and `config.ini` separately so each change has clear evidence.

Use the packaged test cards as examples for the level of evidence expected before calling an airframe ready.
