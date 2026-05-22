# Report v41 - QuadTailsitter qtail7 Orbit Damping

Date: 2026-05-22

## Scope

This report reviews `/home/alireza/qtail7.zip`, which was the first
QuadTailsitter run to include takeoff, hover, Go-To, Orbit, RTL, landing, and
disarm without a crash. The goal is to preserve the improved hover while
reducing the Go-To wandering and the rough initial orbit capture.

## Evidence From qtail7

- px4xplane loaded `v3.4.28`, `Config Name: QuadTailsitter`, and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- PX4 loaded `SYS_AUTOSTART=5021` with the intended qtail defaults:
  `CA_ROTOR*_CT=2.0`, `CA_ROTOR*_KM=+/-0.05`, `MC_YAWRATE_MAX=45`,
  `MPC_XY_CRUISE=1.5`, and `MPC_XY_VEL_MAX=2.0`.
- TruthCapture was healthy: `28,643` frames over about `352.7 s`, zero dropped
  rows, no sim-time resets, mean callback rate about `81.2 Hz`, and max frame
  period about `50 ms`.
- PX4 completed the commanded workflow: arm/takeoff at about `14.4 s`, Go-To
  at `43.9 s`, Orbit at `93.9 s`, RTL at `202.4 s`, landing detection at
  `285.1 s`, and disarm at `286.6 s`.
- Estimator health was not the limiter. In-flight innovation ratios stayed
  low: velocity about `0.44`, position about `0.33`, height about `0.11`, and
  heading about `0.08` worst case.

## Findings

The qtail7 issue is control tracking, not a bridge sensor problem:

- PX4 control allocation reported torque and thrust setpoints achieved during
  Go-To and Orbit.
- Motor outputs had substantial headroom. Orbit motor commands were roughly in
  the `0.17-0.32` range, with no sustained high or low saturation.
- Estimator-vs-truth attitude error stayed small enough for this phase: orbit
  p95 about `1.4 deg` roll, `2.2 deg` pitch, and `3.5 deg` yaw.

The rough orbit entry is dominated by yaw and attitude tracking lag:

- Go-To yaw error was about `9.2 deg` RMS and `22.6 deg` p95.
- Orbit yaw error was about `18.1 deg` RMS and `38.6 deg` p95, with a maximum
  near `74 deg` during the early capture.
- Orbit roll/pitch attitude errors also rose during the early capture, p95
  about `18-20 deg`, then improved after the vehicle converged.
- The Orbit command from QGC used a `30 m` radius and yaw behavior `0`, which
  means the vehicle faces the center by default. PX4's orbit documentation
  confirms this default center-facing behavior, so yaw tracking matters more
  here than in a simple hover.

The connection observation does not point to a new px4xplane simulator-TCP
regression:

- X-Plane `Log.txt` showed the plugin waiting for PX4 and then accepting the
  PX4 simulator connection normally.
- The v3.4.20 nonblocking accepted-socket regression remains reverted. Current
  v3.4.28/v3.4.29 connection code keeps the accepted simulator socket blocking
  and sends complete packets.
- A QGC parameter progress bar stuck mid-load is more likely on the GCS-PX4
  MAVLink/router/cache path than on the px4xplane simulator TCP path. If it
  repeats, collect QGC logs and try clearing QGC's PX4 parameter cache before
  changing plugin connection logic again.

## Motor-Cant Decision

Motor cant remains a valid physical-model experiment, but it is not applied in
this package. The qtail7 log does not show a torque ceiling: allocator achieved
the requested torque and motors had headroom. The safer next step is therefore
a parameter-only yaw and trajectory damping slice.

For a future physical-model slice, a small tangential fixed motor cant should be
tested as a controlled sweep, for example `0/3/5/7 deg`, with matching changes
in the X-Plane ACF thrust axes and PX4 `CA_ROTOR*_AX/AY/AZ` allocation axes.
Do not add cant only in X-Plane or only in PX4; that creates a model/controller
mismatch. Public tailsitter research supports the concept because canted rotors
can increase yaw-control derivative, but qtail7 does not yet require the added
model risk.

References:

- PX4 tailsitter docs:
  https://docs.px4.io/main/en/frames_vtol/tailsitter
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 Orbit flight mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit.html
- PX4 OrbitStatus yaw-behavior message:
  https://docs.px4.io/main/en/msg_docs/OrbitStatus
- Reddinger, Whitt, and Juhasz, "System Identification of a Hovering
  Quadrotor Biplane Tailsitter with Canted Motors":
  https://doi.org/10.1017/aer.2025.35

## Implemented v3.4.29 Changes

- Increased yaw tracking authority while preserving the qtail7 hover baseline:
  `MC_YAWRATE_P=0.065`, `MC_YAWRATE_I=0.015`, and
  `MC_YAWRATE_MAX=60`.
- Slightly reduced rotor yaw moment effectiveness from `CA_ROTOR*_KM=+/-0.05`
  to `+/-0.04`. With `CT=2.0`, this remains above PX4's weak-effectiveness
  cutoff and asks the allocator for more motor differential for a given yaw
  torque demand.
- Increased roll/pitch rate P cautiously from `0.08` to `0.09` because qtail7
  orbit capture showed attitude lag without saturation.
- Reduced low-speed navigation aggressiveness:
  `MPC_XY_CRUISE=1.2`, `MPC_XY_VEL_MAX=1.8`, `MPC_ACC_HOR=0.7`,
  `MPC_ACC_HOR_MAX=1.0`, `MPC_JERK_AUTO=0.8`, and `MPC_JERK_MAX=1.5`.
- Explicitly set `MPC_YAW_MODE=0`, `MPC_YAWRAUTO_MAX=60`,
  `MPC_YAWRAUTO_ACC=20`, `MIS_YAW_ERR=25`, and `MIS_YAW_TMT=5` so this
  airframe no longer inherits the generic VTOL yaw-first behavior silently.
- Left EKF, bridge sensor mapping, X-Plane ACF geometry, mass, fixed-wing
  gains, and transition settings unchanged.

## Next Test

Use v3.4.29 for one multicopter-mode validation:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean` once.
2. Install the v3.4.29 px4xplane package and packaged `QuadTailsitter`
   aircraft.
3. Start XPlaneTruthCapture before connecting PX4.
4. Take off, hold `15-20 s`, command one modest Go-To, command a `30-40 m`
   Orbit, then RTL or Land.
5. Do not command forward transition yet.

Acceptance for qtail8:

- No crash, no attitude failure, and no estimator warnings.
- Go-To yaw p95 below qtail7's `22.6 deg`.
- Orbit yaw p95 clearly below qtail7's `38.6 deg` and early orbit radius p95
  error better than about `5 m`.
- Motors still show headroom and `control_allocator_status` still reports
  torque achieved.
- If yaw remains slow with clean motor headroom, the next slice is the
  controlled canted-motor geometry sweep.
