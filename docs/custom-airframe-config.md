# Defining Custom Airframes in PX4-XPlane

This guide walks you through defining custom airframe configurations in PX4-XPlane using the `config.ini` file. We’ll cover key concepts, provide detailed examples, and guide you through the entire process—from configuring the file to testing your setup in X-Plane.

## Overview

To define a custom airframe configuration, follow these steps:
1. **Select or Build a Plane Model**: Choose an existing X-Plane model or create one from scratch using X-Plane’s Plane Maker.
2. **Select a Similar Airframe in PX4 or Start from Scratch**: Choose a pre-existing PX4 airframe that closely matches your custom setup, or define a new one from scratch. You can double-check channel numbers, servos, and functionalities in the dynamic mixing section of QGroundControl’s Actuator section under SITL.
3. **Configure the `config.ini` File**: Map PX4 channels to X-Plane datarefs.
4. **Understand Datarefs and Their Types**: Learn how to select and configure datarefs.
5. **Build and Test the Configuration**: Test your configuration in X-Plane and PX4.

## Step 1: Select or Build a Plane Model

### Using Existing Models

- **Pre-existing Models**: Use existing X-Plane models that closely match your custom airframe.
- **Modify Models**: Modify an existing X-Plane model to fit your requirements.

### Creating a New Model

- **Use Plane Maker**: Create a new aircraft using X-Plane’s Plane Maker tool if no existing model fits your needs.

## Step 2: Select a Similar Airframe in PX4 or Start from Scratch

### Using Pre-existing Airframes in PX4

- **Choose a Similar Airframe**: If your custom airframe is similar to an existing setup, use that as a starting point (e.g., `xplane_alia250`).
- **Dynamic Mixing in QGroundControl**: Verify and adjust channel mappings in the dynamic mixing section of the Actuator settings in QGroundControl’s SITL section.

### Starting from Scratch

- **Define in PX4 Source Code**: For complex airframes, you may prefer to define the aircraft directly in the PX4 source code and compile it.
- **Dynamic Mixing in QGroundControl**: Verify and adjust channel mappings in the dynamic mixing section of the Actuator settings in QGroundControl’s SITL section.

## Step 3: Configuring the `config.ini` File

The `config.ini` file defines how PX4’s outputs (e.g., motors, control surfaces) map to X-Plane’s inputs.

### Example Configuration for Alia 250 eVTOL

```ini
; PX4-XPlane Configuration File
; File Version: 1.3
; Last Updated: December 26, 2023
; This file configures the mapping of motor and servo channels from PX4 to X-Plane.

; Configuration Name (Displayed in UI)
config_name = Alia250

[Alia250]

; Auto-Prop Brakes: Specifies which motors have an auto-prop brake system.
; Format: Comma-separated list of motor indices in X-Plane.
autoPropBrakes = 0, 1, 2, 3
autoPropBrakeApplyThreshold = 0.01
autoPropBrakeReleaseThreshold = 0.12
autoPropBrakeDwellSec = 2.0
autoPropBrakeMinAirspeedMps = 55.0
autoPropBrakeMode = feather
autoPropBrakeUseFailure = false

; Pitot / differential-pressure source.
; Use xplane_indicated for conventional aircraft when X-Plane IAS has the
; correct sign. Use body_axis for tailsitters or custom pitot placement, then
; set pitotAxisBody to the physical probe axis in PX4 body frame.
airspeedSource = xplane_indicated
pitotAxisBody = +X

; Quadcopter Motors (1-4)
channel0 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]
channel1 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [1], [-1 1]
channel2 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [2], [-1 1]
channel3 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [3], [-1 1]

; Fixed-Wing Control Surfaces (Ailerons, Elevator, Rudder)
channel4 = sim/flightmodel/controls/wing2l_ail1def, float, 0, [-20 10] | sim/flightmodel/controls/wing2l_ail2def, float, 0, [-20 10] | sim/flightmodel/controls/wing3l_ail2def, float, 0, [-20 10]
channel5 = sim/flightmodel/controls/wing2r_ail1def, float, 0, [-20 10] | sim/flightmodel/controls/wing2r_ail2def, float, 0, [-20 10] | sim/flightmodel/controls/wing3r_ail2def, float, 0, [-20 10]
channel6 = sim/flightmodel/controls/hstab1_elv1def, float, 0, [-15 20] | sim/flightmodel/controls/hstab1_elv2def, float, 0, [-15 20]
channel7 = sim/flightmodel/controls/vstab1_rud1def, float, 0, [-15 20] | sim/flightmodel/controls/vstab2_rud2def, float, 0, [-15 20]

; Additional Forward Thrust Motor
channel8 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [4], [-1 1]
```

### Key Sections Explained

- **Configuration Name**: Displayed in the PX4-XPlane UI; this helps you identify the active configuration.
- **Auto-Prop Brakes**: Reduces drag by braking configured X-Plane engine indices when they are not commanded. Use `feather` first; `hard_lock`, `prop_separate`, and `autoPropBrakeUseFailure` are experimental recovery-test options.
- **Airspeed Source**: Selects the simulated pitot source. `xplane_indicated`
  uses X-Plane's IAS dataref, `disabled` sends zero differential pressure, and
  `body_axis` projects the local air-relative velocity onto `pitotAxisBody`.
  Conventional aircraft normally use `+X`; tailsitters may need a different
  physical probe axis such as `-Z`.
- **Channel Mappings**: Defines how each PX4 channel maps to X-Plane’s datarefs. Each channel can control a motor, control surface, or other aircraft function.

Auto-prop brakes are mode-agnostic. The bridge watches the configured motor
commands: it applies brakes only after all listed motors stay below
`autoPropBrakeApplyThreshold` for `autoPropBrakeDwellSec` seconds and the
optional true-airspeed gate is met. Any listed motor above
`autoPropBrakeReleaseThreshold` releases all configured brakes immediately, so
PX4 remains the source of authority during hover, front transition,
back-transition, and assisted recovery.

## Step 4: Understanding Datarefs and Their Types

### Dataref Types

- **`float`**: Single floating-point value, typically used for control surfaces like ailerons or rudders.
- **`floatArray`**: An array of floating-point values, often used for multi-engine aircraft or multiple control surfaces.

Only `float` and `floatArray` are supported by the current runtime parser. Use
external plugin/dataref logic for integer-only systems until the typed config
schema adds explicit integer support.

### Selecting Datarefs

Choosing the correct dataref ensures accurate simulation behavior:
- **Throttle and Motors**: Use `sim/flightmodel/engine/ENGN_thro_use` with `floatArray`.
- **Control Surfaces**: For ailerons, elevators, and rudders, use datarefs like `sim/flightmodel/controls/wing2l_ail1def` with `float`.
- **Auto-Prop Brakes**: Define motor indices that should have auto-prop brakes to minimize drag during flight.

Use the Dataref Tool plugin in X-Plane to find the correct dataref for your setup, or refer to the [X-Plane Dataref Documentation](https://developer.x-plane.com/datarefs/) for more details.

### Runtime Reload and Safety

Mappings, diagnostics, HUD settings, and MAVLink rates can be changed in
`config.ini` and reloaded from the px4xplane menu. If you change the loaded
X-Plane aircraft, aircraft-specific datarefs, or PX4 airframe, reconnect PX4 SITL
and reload the configuration before flying.

px4xplane validates the active `config.ini` with `tools/validate_config.py` and
guards runtime actuator writes:

- stale channel mappings are cleared on every reload
- actuator commands are clamped to finite values before dataref writes
- invalid finite ranges are skipped and logged
- configured actuator datarefs are zeroed if PX4 actuator input becomes stale

Run this before packaging or sharing a custom airframe:

```bash
python3 tools/validate_config.py config/config.ini
```

The validator uses `config/config_schema.json` for global field types, ranges,
and reload policy. To see which fields are safe to reload live and which require
disconnect/reconnect before flight:

```bash
python3 tools/validate_config.py --list-fields
```

Treat actuator mappings, `config_name`, prop-brake motor lists, accelerometer
calibration, and MAVLink target rates as setup-time fields. Change them before
connecting PX4 SITL, or disconnect, reload, validate, and reconnect before
flying. Compact diagnostic and UI fields can be reloaded during a debugging
session, but do not change any config while armed unless you are deliberately
diagnosing a controlled test.

## Step 5: Building and Testing the Configuration

### Testing

1. **Load and Test in PX4-XPlane**: Load the configuration in X-Plane and test the controls.
2. **Adjust as Needed**: Modify the `config.ini` file or PX4 parameters to fine-tune control and simulation accuracy.

### Final Notes: Custom Airframe Setup in PX4

Remember, the channel numbers should be defined in the SITL Actuator section of QGroundControl. You can either:
- **Use Dynamic Mixing**: Adjust actuator outputs directly within QGroundControl for flexibility.
- **Define in Source Code**: For more complex setups, define the airframe in the PX4 source code, build it, and then test.

By following these steps, you’ll ensure your custom airframe behaves correctly in both PX4 and X-Plane, allowing for realistic and accurate simulations.
