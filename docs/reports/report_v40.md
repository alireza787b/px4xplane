# Report v40 - QuadTailsitter qtail6 Hover Yaw Polish

Date: 2026-05-21

## Scope

This report reviews `/home/alireza/qtail6.zip`, the first QuadTailsitter run
that took off, accepted two Go-To commands, landed, and disarmed without a
crash. The goal is to preserve the qtail6 stability gains while reducing the
near-ground takeoff pause and the remaining yaw/attitude wobble.

## Evidence From qtail6

- px4xplane loaded `v3.4.27`, `Config Name: QuadTailsitter`, and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- PX4 loaded `SYS_AUTOSTART=5021` with the intended qtail defaults:
  `CA_ROTOR*_CT=2.0`, `CA_ROTOR*_KM=+/-0.05`, `MC_AIRMODE=2`,
  `MPC_TKO_SPEED=0.6`, and `MPC_TKO_RAMP_T=2.5`.
- TruthCapture was healthy: `16,866` frames, zero dropped rows, no sim-time
  resets, and about `79.8 Hz` mean callback rate.
- PX4 detected takeoff at about `16.1 s`, accepted Go-To commands at about
  `53.7 s` and `84.5 s`, entered Land at about `127.3 s`, detected landing at
  about `161.5 s`, and disarmed at about `163.0 s`.
- Estimator health was clean for this phase: max estimator test ratios were
  about `0.29` velocity, `0.17` position, and `0.07` height.

## Findings

qtail6 confirms the v3.4.27 control-allocation recovery was correct:

- `control_allocator_status` reported torque and thrust achieved throughout
  takeoff, hover, Go-To, and landing.
- `actuator_motors` did not show sustained high or low saturation.
- X-Plane truth showed the current ACF remains a `2.27 kg` aircraft; hover
  thrust was about `22 N`, matching weight, at roughly `0.24` average throttle.

The low just-after-takeoff behavior was commanded, not a bridge sensor bug:

- The local position setpoint stayed near the ground for the first few seconds
  after takeoff, then climbed slowly.
- The active `MPC_TKO_SPEED=0.6` is below PX4's documented normal takeoff climb
  range, and `MPC_TKO_RAMP_T=2.5` makes the ramp intentionally soft. PX4's
  parameter reference also warns that an overly slow ramp can make the vehicle
  scratch the ground or tip over.
- The command target came from QGC's takeoff command and was about `22 m` local
  altitude; `MIS_TAKEOFF_ALT=1.5` was not the controlling value for this run.

The remaining wobble is mainly yaw lag:

- During the first Go-To segment, roll and pitch tracking were acceptable for a
  recovery tune, with RMS errors around `3 deg`, but yaw RMS error was about
  `12.5 deg` and p95 yaw error about `30.7 deg`.
- During the second Go-To segment, yaw improved but still had about `6.9 deg`
  RMS and `18.3 deg` p95 error.
- Position tracking was already usable for a hover-recovery tune: Go-To
  horizontal p95 error was about `3.6 m` then `2.6 m`.

## Motor-Cant Decision

The canted-motor idea is technically valid. PX4 documents quad tailsitters as
supported but harder to tune because of hover/transition aerodynamics, and a
2025 Aeronautical Journal paper on quadrotor biplane tailsitters reports that
rotor cant materially increases yaw control derivative and can improve yaw
control in hover.

Do not add physical cant in this package. qtail6 does not show actuator
saturation or persistent unallocated yaw torque, so the first controlled step
is a param-only yaw tracking polish. Adding motor cant changes both the
X-Plane ACF thrust vectors and the PX4 `CA_ROTOR*_AX/AY/AZ` control-allocation
geometry, which would invalidate the qtail6 comparison.

If qtail7 still shows yaw lag after the param-only polish, the next physical
model slice should add a small tangential cant, about `5 deg`, to the ACF and
the PX4 rotor axes together. Radial inward tilt alone is not enough; the useful
component for yaw is tangential lateral thrust that adds yaw moment in the same
direction as the rotor torque sign.

References:

- PX4 tailsitter docs:
  https://docs.px4.io/main/en/frames_vtol/tailsitter
- PX4 multicopter PID tuning:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 takeoff parameter reference:
  https://docs.px4.io/v1.15/en/advanced_config/parameter_reference.html
- Reddinger, Whitt, and Juhasz, "System Identification of a Hovering
  Quadrotor Biplane Tailsitter with Canted Motors":
  https://doi.org/10.1017/aer.2025.35

## Implemented v3.4.28 Changes

- Raised takeoff climb from `MPC_TKO_SPEED=0.6` to `1.0` and shortened
  `MPC_TKO_RAMP_T` from `2.5` to `1.5` so the aircraft leaves the ground more
  decisively without turning this into an aggressive transition tune.
- Raised vertical auto climb authority:
  `MPC_Z_V_AUTO_UP=1.0`, `MPC_Z_VEL_MAX_UP=1.5`, and
  `MPC_ACC_UP_MAX=2.2`.
- Modestly increased yaw authority after qtail6 proved yaw allocation and
  motor headroom were healthy:
  `MC_YAW_P=0.42`, `MC_YAW_WEIGHT=0.35`, `MC_YAWRATE_P=0.05`,
  `MC_YAWRATE_I=0.010`, and `MC_YAWRATE_MAX=45`.
- Kept roll/pitch rate gains, CT/KM, mass, ACF geometry, horizontal speed,
  tilt limits, fixed-wing settings, and transition settings unchanged.

## Next Test

Use v3.4.28 for one hover and Go-To validation before transition:

1. Install the packaged QuadTailsitter aircraft and plugin.
2. Copy or pull the updated `5021_xplane_qtailsitter`, then run `make
   distclean` once before the test.
3. Start XPlaneTruthCapture before connecting PX4.
4. Take off, let it reach QGC's commanded takeoff altitude, hold for
   `15-20 s`, command two modest Go-To movements, then land.
5. Do not command forward transition yet.

Acceptance for qtail7:

- No crash, no attitude failure, and no estimator warnings.
- Low-after-takeoff pause reduced versus qtail6.
- Yaw p95 error during Go-To clearly below qtail6's `30.7 deg` first-leg value.
- Roll/pitch tracking remains comparable to qtail6 and no sustained motor
  saturation appears.
- If yaw still lags with clean motor headroom, move to the controlled
  `5 deg` canted-motor physical-model slice.
