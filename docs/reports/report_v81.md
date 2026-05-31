# Report v81 - Alia GNSS Position-Only Yaw Reset Regression

Date: 2026-05-31

## Evidence

- `newAlia5` used px4xplane `v3.4.68`, Alia `SYS_AUTOSTART=5020`, and the
  expected current PX4 defaults:
  - `SENS_GPS0_DELAY=10`
  - `SENS_GPS1_DELAY=10`
  - `EKF2_MAG_TYPE=1`
  - `EKF2_GPS_P_NOISE=5.0`
  - `EKF2_GPS_P_GATE=10.0`
  - `EKF2_GSF_TAS=0.0`
- Front transition started at about `125.3 s`. PX4 declared fixed-wing state at
  about `174.1 s`.
- At about `174.5 s`, main EKF yaw, magnetic heading, and GPS course still
  agreed near `-85 deg`. EKF-GSF yaw was near `-54 deg`.
- GNSS velocity fusion was healthy at the reset point. GNSS velocity test ratio
  was about `0.06`; magnetic test ratio was near zero.
- GNSS position fusion had a transition transient and was rejected. Current PX4
  EKF2 allowed that position-only rejection to trigger EKF-GSF yaw rescue, so
  yaw reset by about `30.6 deg` and PX4 reported `Compass 0 fault`.
- The aircraft then recovered, completed the mission, detected landing, and
  auto-disarmed. The warning was therefore an estimator health regression, not a
  flight-control failure in this run.

## Regression Window

The accepted Alia phase predates the PX4-main cleanup. Moving the PR branch onto
current PX4 exposed these relevant EKF2 changes:

- `d62f112017` changed emergency yaw-reset evidence from velocity-fusion timeout
  only to velocity or position innovation rejection.
- `b9699d3009` fixed EKF-GSF GNSS velocity accuracy units, changing the GSF
  estimator's behavior on current logs.
- `cdcdd1096f` added the `in_transition` status flag that v3.4.68 used for the
  first guard.

The v3.4.68 guard was correct but incomplete: `newAlia5` reset immediately after
PX4 left transition, while GNSS velocity was still healthy and EKF-GSF was still
not a safe yaw source.

## Fix

The PX4 PR branch now keeps EKF-GSF yaw rescue enabled for:

- GNSS velocity-fusion rejection while velocity aiding is active.
- GNSS position-fusion rejection only when GNSS velocity aiding is not active.

This keeps the upstream position-only fallback for vehicles or phases without
velocity aiding, but prevents a healthy GNSS velocity solution from being
overridden by a position-only transient during VTOL handoff.

## Next Test Gate

- No `Compass needs calibration` or `Compass 0 fault` during Alia front
  transition.
- No `vehicle_attitude.quat_reset_counter` increment at the fixed-wing handoff
  unless GNSS velocity is also rejected.
- Brief startup `no heading reference` remains acceptable only before
  `Ready for takeoff`.
- Post-landing vertical-speed warnings should be checked against whether the
  aircraft has already reached `landed=true` and disarmed. In `newAlia5`,
  landing detection and auto-disarm succeeded.
