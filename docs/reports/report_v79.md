# Report v79 - Alia Transition Compass Fault Root Cause

## Evidence

- `newAlia3` loaded Alia (`SYS_AUTOSTART=5020`) with current PX4 params, but the
  installed X-Plane plugin was still `v3.4.63`.
- The ULog compass fault started during front transition at about `131 s`.
- `cs_mag_fault` switched on only after an EKF yaw reset. PX4 sets the mag fault
  flag by design when an in-flight GPS emergency yaw reset happens while magnetic
  heading fusion is active.
- GPS velocity innovations stayed small, but horizontal GPS position innovations
  grew beyond the configured gate while the aircraft accelerated through the
  transition.
- EKF airspeed fusion started at about `94 s` with `EKF2_ARSP_THR=10`, long
  before Alia had finished the VTOL transition. This allowed EKF-GSF yaw aiding
  to use transition-state true airspeed before the fixed-wing airframe geometry
  was valid for that assumption.

## Root Cause

The warning was not a raw compass failure. It was a false yaw emergency reset
caused by two simulator-covariance issues happening together:

1. The X-Plane HIL GPS horizontal position covariance was too optimistic for
   high-speed transition and turn phases.
2. EKF airspeed/wind fusion was allowed to start inside the VTOL transition
   window, where the EKF-GSF yaw estimator can be a poor rescue reference.

## Changes

- Added configurable HIL GPS reported accuracy:
  - `gps_horizontal_accuracy_m=1.5`
  - `gps_vertical_accuracy_m=1.0`
- Updated packaged PX4 airframes to use `EKF2_GPS_P_NOISE=5.0` and
  `EKF2_GPS_P_GATE=10.0` for X-Plane HIL.
- Set Alia `EKF2_ARSP_THR=50.0` and `EKF2_GSF_TAS=52.0` so EKF airspeed fusion
  begins after the handoff to fixed-wing flight, while VTOL transition still uses
  the real simulated pitot through PX4 airspeed validation.
- Set QuadTailsitter `EKF2_ARSP_THR=24.0` and `EKF2_GSF_TAS=27.0` for the same
  reason.
- Set fixed-wing-only airframes to use trim-speed `EKF2_GSF_TAS` values and a
  normal `EKF2_ARSP_THR=10.0`.

## Test Focus

For the next Alia run, confirm:

- X-Plane `Log.txt` shows `px4xplane: Version: v3.4.67`.
- PX4 params include `EKF2_ARSP_THR=50`, `EKF2_GPS_P_NOISE=5`, and
  `EKF2_GPS_P_GATE=10`.
- No `Compass 0 fault` appears during front transition.
- Land detection reaches `landed=true` without needing manual disarm, or record
  the exact touchdown window for the next landing-detector-only pass.
