# Report v21 - Alia v3.4.9 Vertical Tune And Baro Source Fix

Date: 2026-05-18

Scope: analyze `/home/alireza/alia-test2.zip`, identify the remaining Alia
vertical-control issues, and prepare the next Alia test package.

## Evidence

The run was valid Alia evidence:

- PX4 launched with `SYS_AUTOSTART=5020`.
- The full mission completed: takeoff, front transition, RTL, back transition,
  landing, disarm, and log close.
- XPlaneTruthCapture recorded `42,344` frames over `546.7 s`, with zero dropped
  rows and no sim-time resets.
- X-Plane callback rate was about `77.5 Hz` mean, `79.7 Hz` median.

The remaining issue is mostly vertical fixed-wing/back-transition tuning, not a
horizontal guidance failure:

- Fixed-wing altitude ranged about `87-149 m` relative altitude.
- Fixed-wing climb rate swung from about `-12.3` to `+12.6 m/s`.
- The worst 10 s fixed-wing altitude swing was about `61.7 m`.
- TECS saturated throttle from `0.10` to `1.00` and pitch from `-5` to `+15 deg`.
- Airspeed was not rejected; `estimator_aid_src_airspeed` max test ratio was
  only about `0.21`.
- GPS height was healthy; GNSS height max test ratio was about `0.50`.

The baro switch was real:

- PX4 logged `[vehicle_air_data] BARO switch from #0 -> #1` around `207.5 s`.
- ULog contained two `sensor_baro` instances with simulated device IDs
  `6620172` and `6620428`.
- `CAL_BARO1_PRIO` was `50` in the ULog, despite the airframe intending backup
  baro priority `0`.

Root cause of the baro priority mismatch:

- PX4 `simulator_mavlink` publishes two simulated baro uORB instances from one
  incoming `HIL_SENSOR` barometer stream.
- The airframe seeded priorities but not device IDs.
- On a fresh or partially reset SITL rootfs, PX4 calibration can auto-create
  both baro calibrations at default priority `50`.
- The user selected `make clean`; that does not guarantee old
  `parameters.bson` calibration state is removed.

## Changes

### px4xplane

- Bumped version to `3.4.9`, build `017`.
- Added prop-brake hysteresis:
  - apply below throttle `0.01`
  - release above throttle `0.05`
  - this reduces failure-dataref chatter around exactly zero throttle.
- Clamped negative indicated airspeed before differential-pressure calculation.
- Kept the `HIL_SENSOR` pressure-altitude bit set intentionally because this
  PX4 `simulator_mavlink` code path uses that bit as part of the baro
  availability mask, even though it derives altitude from `abs_pressure`.

### PX4 X-Plane airframes

For all X-Plane airframes:

- Added simulated baro IDs:
  - `CAL_BARO0_ID=6620172`
  - `CAL_BARO1_ID=6620428`
- Kept:
  - `CAL_BARO0_PRIO=100`
  - `CAL_BARO1_PRIO=0`
- Relaxed `EKF2_BARO_NOISE` from `0.003` to `0.05`.

Alia-specific tuning:

- `FW_T_ALT_TC: 4.0 -> 7.0`
- `FW_T_RLL2THR: 8.0 -> 6.0`
- `FW_T_SINK_MIN: 2.0 -> 1.0`
- `FW_T_SINK_MAX: 4.0 -> 3.0`
- `FW_T_CLMB_MAX: 4.0 -> 3.0`
- `FW_T_VERT_ACC: 4.0 -> 2.5`
- `FW_T_THR_DAMPING: 0.12 -> 0.25`
- `FW_T_PTCH_DAMP: 0.14 -> 0.30`
- `FW_T_I_GAIN_PIT: 0.30 -> 0.15`
- `FW_THR_TRIM: 0.80 -> 0.65`
- `FW_THR_SLEW_MAX: 0.50 -> 0.30`
- `VT_B_TRANS_DUR: 35.0 -> 20.0`
- `VT_B_DEC_MSS: 3.0 -> 2.2`
- `VT_B_DEC_I: 0.40 -> 0.25`
- `RTL_DESCEND_ALT: 300 -> 120`

## Rationale

The fixed-wing controller was not running out of airspeed. It was commanding
alternating max/min energy corrections and the simulated aircraft was
overshooting the requested height-rate by a large margin. The next tune therefore
slows TECS altitude correction, increases damping, reduces pitch integrator
aggression, lowers cruise throttle feed-forward, and limits commanded climb/sink
rates.

The back-transition tune follows PX4 guidance: `VT_B_DEC_MSS` controls expected
deceleration, `VT_B_DEC_I` controls how aggressively pitch-up is used to achieve
it, and high `VT_B_TRANS_DUR` allows more glide with altitude-only control. The
new values reduce the aggressive brake/climb behavior while keeping enough time
to slow down.

References:

- PX4 fixed-wing TECS/NPFG guide:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 parameter reference for `EKF2_BARO_NOISE` minimum `0.01 m`:
  https://docs.px4.io/v1.16/en/advanced_config/parameter_reference

## Next Test Instructions

Use X-Plane 12 Alia and model calculations per frame `6`.

Important: after replacing the PX4 airframe file, choose `d` / `distclean` once
in the setup script, or otherwise delete/reset the PX4 SITL parameter store.
`make clean` alone can preserve `parameters.bson` and mask the new airframe
defaults.

Before takeoff, confirm in the ULog or PX4 shell that these values are active:

- `SYS_AUTOSTART=5020`
- `CAL_BARO0_ID=6620172`
- `CAL_BARO0_PRIO=100`
- `CAL_BARO1_ID=6620428`
- `CAL_BARO1_PRIO=0`
- `EKF2_BARO_NOISE=0.05`
- `FW_T_ALT_TC=7.0`
- `FW_T_THR_DAMPING=0.25`
- `FW_T_PTCH_DAMP=0.30`
- `VT_B_DEC_I=0.25`
- `RTL_DESCEND_ALT=120`

Recommended flight card:

1. Start XPlaneTruthCapture.
2. Start PX4 Alia SITL.
3. Take off to about `100 m`.
4. Forward transition.
5. Fly a circle/loiter segment long enough to expose the old oscillation.
6. Trigger RTL.
7. Observe back-transition overshoot and climb.
8. Land and wait `10-15 s` after disarm before stopping PX4.
9. Save ULog, PX4 terminal, X-Plane `Log.txt`, and the truth-capture run folder.

Acceptance for this slice:

- No in-flight baro source switch.
- No fixed-wing pitch/throttle limit cycling that produces `40-60 m` altitude
  cycles.
- Back-transition peak climb and landing overshoot are lower than the
  `/home/alireza/alia-test2.zip` run.
- Startup-only airspeed selector warning is acceptable if it clears before
  arming and no in-flight airspeed rejection appears.
