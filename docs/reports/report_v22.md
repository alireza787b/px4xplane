# Report v22 - Alia v3.4.10 Regression Recovery

Date: 2026-05-18

Scope: analyze `/home/alireza/alia-test3.zip` and
`/home/alireza/alia-test4.zip`, explain the v3.4.9 regression, and prepare a
safer next Alia test package.

## Evidence

Both runs were valid Alia evidence:

- PX4 launched with `SYS_AUTOSTART=5020`.
- X-Plane loaded `px4xplane v3.4.9 (Alia Vertical Tuning)`.
- TruthCapture recorded clean data:
  - test3: `42,205` rows, zero dropped rows, median FPS about `83.7`.
  - test4: `71,533` rows, zero dropped rows, median FPS about `77.6`.

The v3.4.9 tune was active in both logs:

- `FW_T_ALT_TC=7.0`
- `FW_T_RLL2THR=6.0`
- `FW_T_SINK_MIN=1.0`
- `FW_T_SINK_MAX=3.0`
- `FW_T_CLMB_MAX=3.0`
- `FW_T_VERT_ACC=2.5`
- `FW_T_THR_DAMPING=0.25`
- `FW_T_PTCH_DAMP=0.30`
- `FW_T_I_GAIN_PIT=0.15`
- `FW_THR_TRIM=0.65`
- `FW_THR_SLEW_MAX=0.30`
- `VT_B_TRANS_DUR=20.0`
- `VT_B_DEC_MSS=2.2`
- `VT_B_DEC_I=0.25`
- `RTL_DESCEND_ALT=120`

The user-observed tracking regression is credible:

- Fixed-wing lateral track-error p95 worsened from about `28.9 m` in the
  previous better run to about `31.7 m` in test3 and `45.9 m` in test4.
- Back-transition exited high and relatively fast, then spent a long time in MC
  RTL/landing. Test3 also showed repeated landing/takeoff detections after
  touchdown and did not disarm before log end.
- The v3.4.9 TECS changes reduced some climb-rate extremes but changed too many
  coupled energy and transition parameters at once, so the flight-quality result
  was worse.

## Distclean Accelerometer Warning

The scary test4 warning was not evidence of a new sign/axis bridge failure.
It was caused by the fresh `distclean` parameter state:

- test2/test3 had a saved simulated accel calibration:
  - `CAL_ACC0_PRIO=50`
  - `CAL_ACC0_XOFF=-0.0100629`
  - `CAL_ACC0_YOFF=-0.0362144`
  - `CAL_ACC0_ZOFF=-0.645399`
- test4 after `distclean` had:
  - `CAL_ACC0_PRIO=-1`
  - all accel offsets `0`
- PX4 logged `Preflight Fail: High Accelerometer Bias` before arming, then later
  committed a large simulated accel offset on disarm.

This is now treated as an Alia airframe initialization issue: the airframe seeds
the empirically stable simulated accel offsets so a fresh rootfs behaves like
the known-good baseline.

## Barometer Findings

The v3.4.9 baro device-ID fix is still valid:

- test4 had `CAL_BARO0_PRIO=100` and `CAL_BARO1_PRIO=0`.
- `vehicle_air_data.baro_device_id` stayed on `6620172`.
- The backup baro existed, but the disabled priority prevented device switching.

However, `EKF2_BARO_NOISE=0.05` was still far too confident. PX4's parameter
definition default is `3.5 m`, and test4 saw multi-meter baro innovations and
persistent baro rejection. The next package uses `1.0 m` across X-Plane
airframes: still tighter than PX4 default, but no longer treats frame-limited
X-Plane baro as centimeter-class height data.

## Prop-Brake Findings

TruthCapture confirmed the hover props were windmilling in fixed-wing:

- During fast fixed-wing flight, hover motor throttle was zero.
- The first four prop speeds stayed around `33-35 rad/s`.
- X-Plane still reported non-zero lift-prop thrust/drag torque in those windows.

The old `autoPropBrakes` behavior toggled X-Plane failure datarefs but did not
reliably stop the prop physics/animation. The next plugin keeps the generic
`autoPropBrakes` config field, but its runtime behavior is stronger:

- apply brake below throttle `0.05`
- release above throttle `0.08`
- set seizure failure while braked
- keep prop separation cleared
- command prop mode feather, prop angle `90 deg`, target prop speed `0`, and
  X-Plane prop angular speed `0` while the brake remains active
- restore the previous prop actuator state when releasing the brake

This keeps the policy general: any future airframe can opt in by listing motor
indices in `autoPropBrakes`.

## Changes

### px4xplane

- Bumped version to `3.4.10`, build `018`.
- Reworked prop-brake enforcement as described above.

### PX4 X-Plane Airframes

- Kept simulated baro IDs and backup baro disable:
  - `CAL_BARO0_ID=6620172`
  - `CAL_BARO0_PRIO=100`
  - `CAL_BARO1_ID=6620428`
  - `CAL_BARO1_PRIO=0`
- Changed all X-Plane airframes:
  - `EKF2_BARO_NOISE: 0.05 -> 1.0`

### Alia Airframe

- Added simulated accel calibration defaults:
  - `CAL_ACC0_PRIO=50`
  - `CAL_ACC0_XOFF=-0.0100629`
  - `CAL_ACC0_YOFF=-0.0362144`
  - `CAL_ACC0_ZOFF=-0.645399`
- Reverted v3.4.9 Alia tuning back to the better-tracking baseline:
  - `FW_T_ALT_TC=4.0`
  - `FW_T_RLL2THR=8.0`
  - `FW_T_SINK_MIN=2.0`
  - `FW_T_SINK_MAX=4.0`
  - `FW_T_CLMB_MAX=4.0`
  - `FW_T_VERT_ACC=4.0`
  - `FW_T_THR_DAMPING=0.12`
  - `FW_T_PTCH_DAMP=0.14`
  - `FW_T_I_GAIN_PIT=0.30`
  - `FW_THR_TRIM=0.80`
  - `FW_THR_SLEW_MAX=0.50`
  - `VT_B_TRANS_DUR=35.0`
  - `VT_B_DEC_MSS=3.0`
  - `VT_B_DEC_I=0.40`
  - `RTL_DESCEND_ALT=300`

## References

- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 fixed-wing TECS/position tuning:
  https://docs.px4.io/v1.13/en/config_fw/advanced_tuning_guide_fixedwing.html
- PX4 barometer parameter definition:
  `/home/alireza/PX4-Autopilot-Me/src/modules/ekf2/params_barometer.yaml`
- X-Plane prop pitch/mode behavior:
  https://developer.x-plane.com/article/understanding-override_prop_pitch-and-override_prop_mode/

## Next Test

Use the v3.4.10 package with a PX4 `distclean` once after replacing the airframe
file. Before takeoff, confirm the sanity values in `docs/ALIA_XPLANE12_TEST.md`.

During the run, watch for:

- no preflight high-accelerometer-bias warning after the fresh `distclean`
- no in-flight baro source switch
- no persistent baro fault during fixed-wing/RTL
- hover prop speed near zero during fast fixed-wing flight
- horizontal tracking returned to at least the v3.4.8/test2 level
- vertical oscillation still measured honestly before any new tuning slice

Do not apply another broad tuning sweep. If this package restores the baseline,
the next tuning slice should change one coupled group at a time and compare
against the same takeoff, transition, circle/RTL, back-transition, landing run.
