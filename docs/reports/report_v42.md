# Report v42 - QuadTailsitter qtail8 Agile MC Recovery

Date: 2026-05-22

## Scope

This report reviews `/home/alireza/qtail8.zip`, including the user's live
parameter experiments. The goal is to use those changes as system-identification
evidence, not copy them blindly, and prepare a safer next multicopter-mode
Position/Go-To/Orbit test.

## Package Integrity

The qtail8 zip is only partly usable:

- `Log.txt`, `04_31_13.ulg`, TruthCapture metadata, and TruthCapture summary
  are usable.
- `04_17_10.ulg` has a bad zip CRC and `pyulog` fails to parse it reliably.
- TruthCapture `frames.csv` fails inflate, so detailed truth cross-checks are
  limited to metadata and the partial stream.

Do not use the corrupted first ULog for tuning conclusions. The second ULog is
valid and is the basis for this report.

## Evidence From The Usable ULog

- px4xplane loaded `v3.4.29`, `Config Name: QuadTailsitter`, and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- PX4 loaded `SYS_AUTOSTART=5021`.
- The second log started while the vehicle was already airborne. Initial
  values were not the clean v3.4.29 defaults: `MPC_XY_VEL_MAX=5`,
  `MPC_VEL_MANUAL=5`, `MPC_TILTMAX_AIR=45`, `MC_YAWRATE_K=1.2`, and
  `MC_YAWRATE_D=0.02` were already active.
- Live changes in the second ULog included:
  `MC_YAWRATE_K=1.3`, `MC_YAWRATE_P=0.2`, `MC_YAWRATE_D=0.04`,
  `MC_YAW_P=0.8`, `MPC_XY_P=0.15`, `MPC_Z_P=1.0`,
  `MPC_Z_VEL_P_ACC` changes from `4.0` to `2.0`,
  `MPC_Z_VEL_I_ACC` changes from `1.0` to `0.5`,
  `MPC_Z_VEL_D_ACC=0.5`, and finally `MPC_XY_CRUISE=3.0`.

Before the final Orbit command, the faster Go-To segment was the first useful
agility datapoint:

- Horizontal velocity reached about `2.8 m/s` p95 and `3.4 m/s` max.
- Altitude error stayed acceptable, about `0.6 m` p95.
- Motor outputs still had headroom, but pitch grew large: pitch p95 was about
  `26 deg` and pitch tracking error p95 about `18 deg`.

That means more speed is possible, but the live combination was too aggressive
for a low-altitude Orbit entry.

## Crash Finding

The crash was triggered by the final Orbit command, not by estimator loss:

- `485.341 s`: PX4 received `MAV_CMD_DO_ORBIT`, radius `30 m`, velocity `NaN`,
  yaw behavior `0` center-facing, and altitude about `38.3 m`.
- `485.378 s`: PX4 switched to Orbit.
- Between `488-491 s`, yaw setpoint and actual yaw swung rapidly while the
  aircraft lost altitude. Vertical control then demanded a climb, but the
  vehicle was already descending and motors 2/3 climbed toward saturation.
- `490.673 s`: first motor hit `1.0`.
- `490.810 s`: allocator reported thrust not achieved.
- About `491.36 s`: estimated altitude crossed the ground/reference region.
- `492.97 s`: PX4 reported attitude failure, roll. At failure, roll was about
  `-90 deg`, yaw error about `68 deg`, and motor 2 was saturated.

Estimator health remained good until after the impact dynamics. The failure was
a control/trajectory/aggressiveness problem: low-altitude center-facing Orbit
with high tilt/speed/yaw authority created a yaw/vertical coupling transient
that the aircraft could not arrest before ground contact.

## Connection And Vertical-Warning Notes

X-Plane `Log.txt` shows normal px4xplane server setup and PX4 simulator
connection on port `4560`. The old v3.4.20 nonblocking accepted-socket
regression was not reintroduced.

The usable ULog does not contain a vertical-velocity warning. Its estimator
vertical velocity validity remains good. TruthCapture metadata shows healthy
timing, zero dropped rows, and about `81 Hz` median frame rate. If the
post-landing vertical warning happened in the corrupted first ULog, it cannot
be proven from this package; based on previous logs, it is still most likely a
ground-idle/landed-state issue rather than a bridge sensor corruption.

## Implemented v3.4.30 Changes

This slice promotes the useful live-test direction while backing away from the
crash-causing combination:

- Raise speed from recovery values but not to the live crash envelope:
  `MPC_XY_CRUISE=3.0`, `MPC_XY_VEL_MAX=3.5`, `MPC_VEL_MANUAL=3.0`,
  `MPC_ACC_HOR=2.0`, `MPC_ACC_HOR_MAX=2.0`, `MPC_JERK_AUTO=1.2`, and
  `MPC_JERK_MAX=2.0`.
- Raise tilt authority to `25 deg`, not the live `45 deg`.
- Use moderate yaw gains:
  `MC_YAW_P=0.55`, `MC_YAWRATE_P=0.10`, `MC_YAWRATE_I=0.015`,
  `MC_YAWRATE_D=0.005`, `MC_YAWRATE_K=1.10`, and `MC_YAWRATE_MAX=60`.
- Slow automatic yaw setpoint motion for Orbit and Auto:
  `MPC_YAWRAUTO_MAX=35`, `MPC_YAWRAUTO_ACC=12`, `MIS_YAW_ERR=30`.
- Set `MC_ORBIT_YAW_MOD=1` so Orbit commands that leave yaw behavior unchanged
  use hold-initial-heading. QGC's default center-facing `param3=0` still
  overrides this, so the next manual Orbit test should use hold initial heading
  or tangent heading if QGC exposes the option.
- Improve altitude hold moderately without copying the aggressive live
  sequence: `MPC_Z_P=0.80`, `MPC_Z_VEL_P_ACC=2.5`,
  `MPC_Z_VEL_I_ACC=0.6`, and `MPC_Z_VEL_D_ACC=0.4`.

No bridge sensor mapping, connection code, X-Plane aircraft geometry, motor
cant, fixed-wing, or transition values are changed in this slice.

## Next Test

Use v3.4.30 for one multicopter-mode validation only:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean` once.
2. Install the v3.4.30 px4xplane package and packaged `QuadTailsitter`
   aircraft.
3. Start XPlaneTruthCapture before connecting PX4.
4. Take off, hold `15-20 s`, command one modest Go-To, then command one faster
   Go-To around `2.5-3.0 m/s`.
5. Start Orbit only at `20 m` AGL or higher. Use `40-50 m` radius. If QGC lets
   you choose yaw behavior, use hold initial heading or tangent heading.
6. Do not command forward transition yet.

Acceptance for qtail9:

- No crash, no attitude failure, and no estimator warnings.
- Go-To reaches roughly `2.5-3.0 m/s` without sustained pitch/roll error.
- Orbit entry does not show the qtail8 yaw spin, vertical sink, motor
  saturation, or thrust-not-achieved sequence.
- If yaw/Orbit still fails with motor headroom, stop increasing gains and move
  to the controlled canted-motor/allocator-axis physical-model sweep.

References:

- PX4 Orbit flight mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit.html
- PX4 Orbit yaw behavior message:
  https://docs.px4.io/main/en/msg_docs/OrbitStatus
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 parameter reference:
  https://docs.px4.io/main/en/advanced_config/parameter_reference.html
