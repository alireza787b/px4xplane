# Report v70 - TB2 Baseline and Cessna Closure Polish

Date: 2026-05-28  
Package target: px4xplane v3.4.58  
Primary goals: close the accepted Cessna slice and prepare the first refreshed TB2 test.

## Cessna Closure

The accepted Cessna package already smoothed elevator as well as ailerons:

```text
actuatorSmoothingTimeConstantSec = 0.04
actuatorSmoothingChannels = 0, 1, 2, 4
```

That covers:

- channel 0: left aileron
- channel 1: right aileron
- channel 2: elevator
- channel 4: rudder

Throttle, nosewheel steering, and flaps remain direct so takeoff/rollout and
landing configuration are not delayed. For the final Cessna polish, landing
airspeed moved from `33.0` to `32.5 m/s`, flare minimum pitch moved from
`8.0` to `9.0 deg`, and `LNDFW_AIRSPD_MAX` was restored to the current PX4
metadata maximum of `30.0`. This is intentionally small because the last Cessna
test was accepted.

## TB2 Evidence Review

The uploaded aircraft package is `/home/alireza/Bayraktar_tb2.zip`. The active
ACF is `Bayraktar_tb2/Bayraktar_tb2.acf`, an X-Plane 12-format model.

Observed ACF evidence:

- Name: `Bayraktar BT2`
- Main-wing span from Plane Maker segments: about `10.8 m`
- Engine: one 100 hp internal combustion engine
- Nose gear: `20 deg` steering available
- Flaps: wing left/right `flap1` surfaces with 10, 30, and 40 deg detents
- A-tail: left/right horizontal-stabilizer elevator surfaces
- Weight fields imply the loaded X-Plane model is much lighter than published
  TB2 MTOW, roughly `418 kg` empty-plus-full-fuel if interpreted in X-Plane's
  aircraft weight units.

Baykar's official page lists TB2 references used for the envelope: `12 m`
wingspan, `6.5 m` length, `2.2 m` height, `700 kg` MTOW, `100 hp` power plant,
`20+ hours` endurance, `90-110 KTAS` travel/max speed, and autonomous
takeoff/landing/taxi capability.

## TB2 Changes

Bridge config:

- Made `TB2` the packaged active airframe for the next test.
- Added explicit conventional pitot settings:
  - `airspeedSource = xplane_indicated`
  - `pitotAxisBody = +X`
- Added scoped smoothing only for aerodynamic surfaces:
  - `actuatorSmoothingTimeConstantSec = 0.04`
  - `actuatorSmoothingChannels = 0, 1, 2, 3`
- Changed nosewheel steering to the Cessna-proven XP12 mapping:
  - `sim/flightmodel2/gear/tire_steer_command_deg[0]`
  - `sim/flightmodel/parts/tire_steer_cmd[0]`
- Changed steering range to `[-20, +20]` to match the ACF nose gear.
- Reordered flap channels so PX4 left/right flap outputs match left/right
  X-Plane surfaces.
- Added `flap_handle_request_ratio` writes so the cockpit handle and physical
  surfaces move together in X-Plane 12.

PX4 airframe:

- Rewrote `5002_xplane_tb2` into the same clean section structure as the
  refreshed Cessna file.
- Replaced the old generic wheel servo with `PWM_MAIN_FUNC6=440`
  (`Landing_Gear_Wheel`) and enabled `FW_W_EN=1`.
- Removed the stale steering-wheel control surface from control allocation;
  CA now owns only ailerons, A-tail surfaces, and flaps.
- Updated simulator sensor settings to the current bridge contract:
  - `IMU_GYRO_RATEMAX=200`
  - `IMU_INTEG_RATE=200`
  - `EKF2_ABL_LIM=0.8`
  - `ASPD_DO_CHECKS=1`
  - `ASPD_FALLBACK=1`
  - `SYS_HAS_NUM_ASPD=0`
- Lowered the old TB2-derived surface feed-forward values that caused Cessna
  visible surface stepping.
- Set first-pass TB2 speed targets around the model and official references:
  - `FW_AIRSPD_TRIM=36 m/s`
  - `FW_AIRSPD_MIN=24 m/s`
  - `FW_AIRSPD_MAX=56 m/s`
  - `FW_LND_AIRSPD=28 m/s`
- Set `WEIGHT_BASE/GROSS=418 kg` and `FW_WING_SPAN=10.8 m` for this first test
  to match the uploaded X-Plane model rather than the published 700 kg MTOW.

## Why This Is Conservative

PX4's fixed-wing guide emphasizes that TECS tuning depends on correct airframe
limits and a working attitude controller. The TB2 pass therefore fixes the
known stale plumbing and uses a moderate first-pass control tune. It does not
claim final TB2 tuning until the first refreshed ULog confirms signs, surface
authority, trim throttle, and runway behavior.

## Next Test

Run `make px4_sitl_default distclean` before testing TB2 so stale
`parameters.bson` does not override the refreshed airframe defaults.

Send back:

- TB2 ULog
- PX4 terminal log
- X-Plane `Log.txt`
- XPlaneTruthCapture recording

Key acceptance checks:

- no preflight high accelerometer bias warning
- no airspeed selector module warning after simulator connection settles
- centered runway takeoff
- visible flaps on landing
- no aggressive surface stepping
- stable cruise/loiter at about 36 m/s target airspeed
- landing flare and rollout remain controlled

## References Used

- Baykar TB2 official product page: https://baykartech.com/en/uav/bayraktar-tb2/
- PX4 fixed-wing altitude/position tuning guide: https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 fixed-wing landing guide: https://docs.px4.io/v1.12/en/flying/fixed_wing_landing
- X-Plane developer note on correct wing datarefs: https://developer.x-plane.com/2013/05/using-the-right-wing-datarefs/
