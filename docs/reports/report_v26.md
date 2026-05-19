# Report v26 - Alia Final Polish and Ehang Prep

Date: 2026-05-18

Scope: review `/home/alireza/alia-test8.zip`, make final Alia polish after the
first broadly accepted v3.4.13 flight, prepare Ehang 184 for a controlled
validation run, and keep both PX4 and plugin-reference airframe files aligned.

## Alia Test8 Evidence

The run used the intended package and branch:

- X-Plane loaded `px4xplane v3.4.13`.
- PX4 ULog reported `SYS_AUTOSTART=5020`.
- PX4 software branch was `px4xplane-sitl` at commit `42f568f`.
- XPlaneTruthCapture recorded `39,657` frames over `562.3 s`, with zero dropped
  rows and no sim-time reset.
- ULog had no dropouts and no in-flight EKF, GPS, baro, airspeed, or quad-chute
  warning messages.

The v3.4.13 orbit recovery worked well enough to be treated as the new Alia
baseline. Remaining issues are polish items: landing took too long from high
RTL descent altitude, and default fixed-wing loiter radius should be larger for
this large eVTOL.

## Alia Final Polish

Landing data showed RTL land began around `165 m` AGL and landing detection
occurred about `141 s` later. PX4 land logic descends at the normal down-speed
above `MPC_LAND_ALT1`, ramps between `MPC_LAND_ALT1` and `MPC_LAND_ALT2`, and
uses `MPC_LAND_SPEED` below `MPC_LAND_ALT2`; with valid range data it uses
`MPC_LAND_CRWL` below `MPC_LAND_ALT3`.

The v3.4.14 profile therefore changes Alia from a long gentle descent to a
faster high-altitude descent with slow final touchdown:

- `MPC_Z_VEL_MAX_DN=2.2`
- `MPC_Z_V_AUTO_DN=2.2`
- `MPC_LAND_ALT1=8`
- `MPC_LAND_ALT2=3`
- `MPC_LAND_SPEED=0.6`
- `MPC_LAND_CRWL=0.25`
- `RTL_DESCEND_ALT=80`

For fixed-wing loiter, v3.4.13 still commanded the configured `22 deg` roll
limit for much of the orbit. That value worked, but PX4 metadata declares
`FW_R_LIM` valid from `35 deg` upward. v3.4.14 moves back to the PX4-valid
minimum and increases fixed-wing loiter radii so the intended steady turn stays
comfortable:

- `FW_R_LIM=35`
- `NAV_LOITER_RAD=1500`
- `RTL_LOITER_RAD=1500`

## Ehang 184 Prep

No new Ehang flight log was available in this slice, so this is an offline
airframe hygiene and first-test preparation pass, not a claim that Ehang is
finished. The file now uses conservative, PX4-valid large-multicopter values:

- `IMU_GYRO_RATEMAX=200`, `IMU_INTEG_RATE=200`
- `MPC_ACC_HOR=2.0`, `MPC_ACC_HOR_MAX=2.0`
- `MPC_ACC_UP_MAX=2.0`, `MPC_ACC_DOWN_MAX=2.0`
- `MPC_JERK_AUTO=1.0`, `MPC_JERK_MAX=2.0`
- `MPC_MAN_TILT_MAX=25`, `MPC_TILTMAX_AIR=30`
- `MPC_MAN_Y_MAX=15`, `MPC_YAWRAUTO_MAX=20`
- `MPC_TKO_SPEED=1.2`
- `MPC_Z_V_AUTO_UP=2.0`, `MPC_Z_V_AUTO_DN=1.0`
- `MPC_LAND_SPEED=0.6`, `MPC_LAND_ALT1=8`, `MPC_LAND_ALT2=3`
- `NAV_ACC_RAD=25`, `NAV_MC_ALT_RAD=2`

Comments were cleaned to remove unsupported production/passenger-operation
certainty and to match the Alia sensor-fusion lesson: X-Plane baro data should
not be treated as centimeter-class when frame timing and simulator physics vary.

## Packaging Notes

- Added `docs/EHANG184_XPLANE12_TEST.md`.
- Packaged releases now include both Alia and Ehang validation cards.
- Moved tracked root reports v12/v13 into `docs/reports`.
- Removed a tracked Windows command-prompt temp file.

## References

- PX4 multicopter land mode:
  https://docs.px4.io/main/en/flight_modes_mc/land.html
- PX4 multicopter trajectory tuning:
  https://docs.px4.io/main/en/config_mc/mc_trajectory_tuning
- PX4 parameter reference:
  https://docs.px4.io/main/en/advanced_config/parameter_reference.html
- PX4 land setpoint source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp`
- PX4 fixed-wing roll limit metadata:
  `/home/alireza/PX4-Autopilot-Me/src/modules/fw_mode_manager/fw_mode_manager_params.c`
