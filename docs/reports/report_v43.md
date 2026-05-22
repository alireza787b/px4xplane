# Report v43 - QuadTailsitter qtail9 Go-To Smoothness

Date: 2026-05-22

## Scope

This report reviews `/home/alireza/05_30_57.ulg`, the first strongly
successful QuadTailsitter multicopter-mode validation after v3.4.30. The user
reported the best flight so far, with one remaining issue: Go-To accelerated,
nearly stopped, then continued instead of moving as one smooth trajectory.

## Log Integrity

The ULog is valid and usable:

- Duration: about `366.9 s`.
- Topics: full estimator, control, actuator, command, and vehicle state set.
- Flight completed takeoff, Go-To, Orbit, RTL, landing, and disarm.
- No evidence of a bridge connection, GPS, or estimator-rooted in-flight
  failure.

## Important Parameter Context

This run was not a clean v3.4.30 default run. Persistent/manual values were
already active at boot:

- `MPC_XY_VEL_MAX=5`, `MPC_VEL_MANUAL=5`, `MPC_TILTMAX_AIR=45`
- `MPC_XY_P=0.15`, `MPC_Z_P=1.0`
- `MPC_Z_VEL_P_ACC=2.0`, `MPC_Z_VEL_I_ACC=0.5`,
  `MPC_Z_VEL_D_ACC=0.5`
- `MC_YAW_P=0.8`, `MC_YAWRATE_P=0.2`, `MC_YAWRATE_D=0.04`,
  `MC_YAWRATE_K=1.2`
- `MC_ROLLRATE_K=1.0`, `MC_PITCHRATE_K=1.0`

Those values are useful system-identification evidence, but should not be
copied blindly into the airframe file.

## Go-To Finding

The Go-To brake/continue behavior is visible in the ULog, but most of it is
expected PX4 point-reposition behavior:

- During the first Go-To, actual horizontal speed reached about `3.1 m/s`.
- The trajectory setpoint then deliberately ramped down near the target.
- The second Go-To arrived while the aircraft was slowing at the first target,
  so it accelerated again.
- This is expected for `DO_REPOSITION`: it is a point-hold command, not a
  blended waypoint path.
- Yaw error stayed small in the first Go-To and roll tracking was acceptable.
- Motors had large headroom; the maximum motor output was only about `0.32`
  during the first Go-To.
- A secondary tuning issue remains. In parts of the Go-To, actual speed fell
  below demanded speed while pitch lagged pitch setpoint by about `15-19 deg`.

This is therefore not an EKF, GPS, or px4xplane sensor-contract issue. The
remaining tunable part is pitch/trajectory tracking, while the final slowdown
near each Go-To point is normal PX4 behavior.

PX4 source and parameter semantics support this diagnosis:

- Autonomous Go-To/mission motion uses `MPC_XY_CRUISE`,
  `MPC_XY_VEL_MAX`, `MPC_ACC_HOR`, and `MPC_JERK_AUTO`.
- `MPC_XY_ERR_MAX` controls how far the trajectory setpoint is allowed to run
  ahead of the vehicle. If tracking error exceeds this limit, PX4 slows or
  stops setpoint integration to let the vehicle catch up, so lowering it here
  would risk adding more stop/wait behavior.
- Orbit has its own velocity and acceleration caps and defaults to a slow
  `1 m/s` orbit unless commanded otherwise.

## Orbit Finding

The qtail9 Orbit segment was much better than qtail8:

- Orbit attitude tracking was controlled.
- Yaw error was small.
- Motor saturation did not drive the behavior.
- No qtail8-style yaw spin, vertical sink, or thrust-not-achieved sequence was
  present.

This supports staying with a parameter-only MC polish and deferring motor cant,
aircraft geometry, fixed-wing, and transition changes.

## Implemented v3.4.31 Changes

v3.4.31 promotes the successful higher-speed direction in a guarded way:

- Speed/trajectory:
  `MPC_XY_CRUISE=5.0`, `MPC_XY_VEL_MAX=5.0`, `MPC_VEL_MANUAL=4.0`,
  `MPC_ACC_HOR=2.2`, `MPC_ACC_HOR_MAX=2.5`, `MPC_JERK_AUTO=1.5`,
  `MPC_JERK_MAX=2.5`.
- Roll/pitch authority:
  `MC_ROLL_P=0.9`, `MC_PITCH_P=0.9`, `MC_ROLLRATE_P=0.10`,
  `MC_PITCHRATE_P=0.10`, `MC_ROLLRATE_D=0.0008`,
  `MC_PITCHRATE_D=0.0008`, `MC_ROLLRATE_K=1.00`,
  `MC_PITCHRATE_K=1.00`, `MC_ROLLRATE_MAX=80`, and
  `MC_PITCHRATE_MAX=80`.
- Yaw authority:
  `MC_YAW_P=0.75`, `MC_YAWRATE_P=0.16`, `MC_YAWRATE_D=0.015`, and
  `MC_YAWRATE_K=1.15`.
- Tilt authority:
  `MPC_TILTMAX_AIR=32`, `MPC_MAN_TILT_MAX=32`.

The live `45 deg` tilt and yaw-rate `D=0.04` were not copied directly because
qtail8 showed that this envelope can still create unsafe Orbit-entry coupling
when paired with a small center-facing orbit.

## Deferred

No px4xplane bridge, connection workflow, sensor contract, X-Plane aircraft
geometry, canted-motor, fixed-wing, or transition changes are made in this
slice. Those remain separate controlled experiments after multicopter
Position/Go-To/Orbit is clean.

## Next Test

Use v3.4.31 for one MC-only validation:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean` once.
2. Install the v3.4.31 px4xplane package and packaged `QuadTailsitter`
   aircraft.
3. Start XPlaneTruthCapture if possible.
4. Take off and hold.
5. Command a `3 m/s` Go-To, then a `4 m/s` Go-To.
6. Only if those are clean, command a `5 m/s` Go-To.
7. Command a `40-50 m` Orbit at `20 m` AGL or higher.
8. Do not command forward transition yet.

Acceptance:

- No unexpected stop/wait while still far from the Go-To point.
- Smooth deceleration at the final Go-To target is acceptable.
- No sustained pitch or roll tracking error.
- No motor saturation or thrust-not-achieved sequence.
- Orbit remains controlled with no qtail8 yaw spin.
- Land detection and disarm remain normal.

References:

- PX4 Orbit flight mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit.html
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 parameter reference:
  https://docs.px4.io/main/en/advanced_config/parameter_reference.html
