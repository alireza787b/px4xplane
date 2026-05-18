# Report v23 - Alia Test5 Crash Recovery

Date: 2026-05-18

Scope: analyze `/home/alireza/alia-test5.zip`, identify the crash root cause,
and prepare the v3.4.11 recovery package.

## Evidence

The test was valid evidence:

- X-Plane loaded `px4xplane v3.4.10 (Alia Regression Recovery)`.
- XPlaneTruthCapture loaded correctly and recorded `15,038` rows with zero
  dropped rows.
- PX4 launched `SYS_AUTOSTART=5020`.
- The ULog had no dropouts.

Timeline from ULog and TruthCapture:

- `~93.3 s`: PX4 entered `TRANSITION_TO_FW`.
- No `FW` VTOL state was ever reached.
- Calibrated airspeed crossed the raw `VT_ARSP_TRANS=48 m/s` gate, but the PX4
  weight scaling made the effective gate about `51.0 m/s`.
- Test5 peak calibrated airspeed was about `50.6 m/s`, so the front transition
  could not complete.
- `~152.6 s`: local altitude fell below `VT_FW_MIN_ALT=20 m`.
- PX4 quad-chuted due to minimum altitude breach, then entered failsafe/RTL.

Estimator health was not the root cause:

- Baro and airspeed aiding were fused and not rejected at the trigger.
- Baro source stayed on the intended primary device.
- Groundtruth also showed low altitude, so the altitude breach was real.

## Root Cause

The crash was caused by two interacting issues.

First, v3.4.10 prop-brake enforcement was unsafe for a VTOL transition. It used
per-motor throttle thresholds, feathered/stopped individual lift props, and used
X-Plane seizure failure while braked. TruthCapture showed brake chatter and
asymmetric lift-prop recovery: some motors were still feathered or nearly stopped
while PX4 was commanding recovery thrust.

Second, Alia's front transition had too little completion margin. With
`VT_F_TRANS_DUR=60 s`, the tiltrotor P1 gate could not complete until near the
same time the aircraft was already close to `VT_FW_MIN_ALT`. With
`WEIGHT_BASE=3100` and `WEIGHT_GROSS=3500`, `VT_ARSP_TRANS=48` became an
effective transition threshold of about `51.0 m/s`, which test5 never reached.

The terminal PX4 trigger was therefore correct: the vehicle really breached the
minimum fixed-wing altitude while still in front transition. The underlying
reason was our bridge brake-policy regression plus insufficient transition
margin.

## v3.4.11 Changes

### Bridge

- Disabled Alia `autoPropBrakes` by default for the next retest.
- Reworked prop-brake policy so future opt-in braking:
  - applies only after all configured motors stay below the apply threshold,
  - releases all brakes immediately if any configured motor requests recovery,
  - uses a dwell timer before applying,
  - supports an optional true-airspeed apply gate,
  - does not use X-Plane seizure failure unless explicitly enabled.
- Added cleanup so disconnect/stale actuator reset releases active prop brakes.
- Restores all prop actuator datarefs that the brake enforcement writes.

### Alia PX4 Airframe

- Changed `VT_ARSP_TRANS` from `48.0` to `46.0`. With the current mass scaling,
  that is an effective gate of about `49 m/s`, which test5 did reach before the
  altitude breach.
- Changed `VT_F_TRANS_DUR` from `60.0` to `45.0`.
- Changed `VT_F_TR_OL_TM` from `70.0` to `55.0`.
- Updated the seeded simulated accel offsets to the values PX4 had learned by
  the no-high-accel-warning test5 start:
  - `CAL_ACC0_XOFF=0.0978824`
  - `CAL_ACC0_YOFF=0.0953933`
  - `CAL_ACC0_ZOFF=-1.2080466`

No broad TECS, NPFG, attitude-rate, or back-transition tuning is included in
this recovery slice.

## Why Not Re-Enable Brakes Yet

TruthCapture proved v3.4.10 stopped windmilling, but it also proved that a
stopped/feathered lift prop can be needed again before PX4 reaches fixed-wing or
while quad-chute/back-transition recovery begins. Normal release packages should
not use prop braking on Alia until mode-aware or externally commanded brake
logic is validated with replay plus SITL.

## Next Test

Use the v3.4.11 package with `distclean` once after replacing the PX4 Alia
airframe file.

Watch specifically for:

- no quad-chute,
- VTOL state reaches fixed-wing shortly after transition airspeed is achieved,
- no high-accelerometer-bias warning from a fresh parameter store,
- no baro switch,
- lift props keep recovering normally during transition and any abort,
- vertical tracking remains measurable for the next narrow tuning slice.

## References

- PX4 VTOL transition behavior and quad-chute checks:
  `/home/alireza/PX4-Autopilot-Me/src/modules/vtol_att_control`
- PX4 VTOL QuadPlane configuration guide:
  https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration.html
- BETA ALIA production page for high-level aircraft context:
  https://www.beta.team/aircraft/
