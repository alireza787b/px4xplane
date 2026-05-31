# Report v80 - Alia VTOL Transition EKF Guard

Date: 2026-05-31

## Evidence

- `newAlia4` contained two Alia runs: Mehrabad at about 45 Hz and Kish at about
  71 Hz. Both loaded `SYS_AUTOSTART=5020` and the v3.4.67 EKF parameter set.
- In both ULogs, `cs_mag_fault` appeared only after `vehicle_attitude`
  yaw-reset counter increments during front transition.
- Magnetometer strength and inclination matched the EKF reference values at both
  airports. Heading innovation stayed low, and no raw mag axis fault flags were
  set.
- The EKF-GSF yaw estimate diverged from the magnetometer/GNSS-consistent main
  EKF yaw during transition, then PX4 reset yaw by about 25-50 degrees and
  marked the compass faulty as part of the emergency yaw reset path.
- The Mehrabad run also showed a stationary-ground preflight vertical velocity
  warning. The X-Plane log showed the ground sensor contract latched and then
  released early while the aircraft was still on the ground.

## Changes

- PX4 EKF2 now refuses EKF-GSF yaw resets while `in_transition` is true. This
  keeps the rescue path available for normal flight while avoiding a known
  non-coordinated VTOL transition regime.
- Alia and QuadTailsitter set `EKF2_GSF_TAS=0` so default EKF-GSF fixed-wing
  centripetal compensation is not applied before validated airspeed fusion is
  active and stable.
- The px4xplane ground-stationary contract now trusts configured PX4 motor
  mappings first. X-Plane idle/contact engine output no longer releases the
  zero-motion sensor contract when PX4 is still commanding zero.
- Alia now uses `LNDMC_Z_VEL_MAX=0.25`, matching PX4's enforced land-detector
  limit for `MPC_LAND_CRWL=0.30`.

## Next Test Gate

- X-Plane `Log.txt` should show px4xplane `v3.4.68`.
- Alia startup should no longer print the `LNDMC_Z_VEL_MAX` auto-rewrite error.
- A mission front transition should not print `Compass needs calibration` or
  `Compass 0 fault`.
- A brief startup `no heading reference` message is acceptable only before
  `Ready for takeoff`; it should not persist.
