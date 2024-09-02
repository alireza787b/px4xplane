# Defining Custom Airframes in PX4-XPlane

This guide will walk you through defining custom airframe configurations in PX4-XPlane using the `config.ini` file. We’ll cover everything from setting up the configuration file to mapping channels in PX4, ensuring you can successfully simulate and control your custom airframes.

## Overview

To define a custom airframe configuration, follow these steps:
1. **Configure the `config.ini` File**: Map PX4 channels to X-Plane datarefs.
2. **Understand Datarefs and Their Types**: Learn how to choose and configure datarefs.
3. **Set Channel Numbers in PX4**: Define channel numbers in the PX4 SITL section via QGroundControl.
4. **Use or Create an Airframe in PX4**: Set up your airframe using dynamic mixing in QGroundControl or define it in the PX4 source code.
5. **Build and Test the Configuration**: Test your configuration in X-Plane and PX4.

## Step 1: Configuring the `config.ini` File

The `config.ini` file is where you define how PX4's outputs are mapped to X-Plane's inputs, such as motors and control surfaces.

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
- **Auto-Prop Brakes**: Reduces drag by braking the propellers when they’re not needed. This is particularly useful for eVTOL configurations.
- **Channel Mappings**: Defines how each PX4 channel maps to X-Plane’s datarefs.

## Step 2: Understanding Datarefs and Their Types

### Dataref Types

- **`float`**: Single floating-point value, used for controls like throttle or rudder.
- **`floatArray`**: Array of floating-point values, often used for engines or multi-servo setups.
- **`int`**: Integer values, typically used for binary states like landing gear.
- **`intArray`**: Array of integers, similar to `floatArray` but for integers.

### Selecting Datarefs

Choose the appropriate dataref based on the control type:
- **Throttle and Motors**: Use `sim/flightmodel/engine/ENGN_thro_use` with `floatArray`.
- **Control Surfaces**: Use datarefs like `sim/flightmodel/controls/wing2l_ail1def` with `float` for ailerons, elevators, and rudders.
- **Auto-Prop Brakes**: If needed, define the motor indices that should have auto-prop brakes to reduce drag.

## Step 3: Setting Channel Numbers in PX4

### Defining Channels in PX4’s SITL Section

1. **Open QGroundControl**: Connect to your PX4 instance.
2. **Navigate to the SITL Actuator Section**: In QGroundControl, go to the "Setup" tab and find the "Actuators" section.
3. **Map Channels**: Ensure the channels you define in the `config.ini` file match the actuator outputs in the SITL section.

### Alternative: Define Aircraft in PX4 Source Code

If your airframe is complex, you may prefer to define the aircraft directly in the PX4 source code and compile it yourself. This involves creating or modifying an airframe definition file in the PX4 Firmware and recompiling PX4.

## Step 4: Building and Testing the Configuration

### Using Pre-existing Airframes

If your airframe closely matches an existing setup, such as `xplane_alia250`, you can use that configuration.

### Creating a Custom Airframe

1. **Modify X-Plane Models**: Use or modify an existing X-Plane aircraft model to match your configuration.
2. **Create New Models in Plane Maker**: If necessary, design a new aircraft in X-Plane’s Plane Maker to match your custom configuration.

### Testing

1. **Load and Test in PX4-XPlane**: Load the configuration in X-Plane and test the controls.
2. **Adjust as Needed**: Make any necessary adjustments in the `config.ini` file or PX4 parameters to ensure proper control and simulation accuracy.

## Final Notes: Custom Airframe Setup in PX4

Remember, the channel numbers should be defined in the SITL Actuator section of QGroundControl. You can either:
- **Use Dynamic Mixing**: Adjust the actuator outputs directly within QGroundControl for flexibility.
- **Define in Source Code**: For more complex setups, define the airframe in the PX4 source code, build it, and then test.

This approach allows you to fine-tune your airframe's behavior within PX4, ensuring that it matches the corresponding X-Plane model.

