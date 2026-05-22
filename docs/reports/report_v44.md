# Report v44 - QuadTailsitter qtail9 Transition Airspeed Recovery

Date: 2026-05-22

## Scope

This report reviews `/home/alireza/qtail9.zip`, which contains the PX4 ULog
`06_52_20.ulg` and XPlaneTruthCapture run `20260522-065213Z`. The user reported
three main symptoms:

- Go-To accelerated, slowed strongly near about `3.7 m/s`, then accelerated
  again.
- During forward transition, QGC heading/vehicle attitude looked unstable.
- Airspeed showed zero and every transition attempt ended in timeout,
  altitude-loss, or quad-chute.

## Log Integrity

The archive is valid:

- Zip test: OK.
- ULog duration: about `650.8 s`.
- Truth capture: `54,342` frames, `0` dropped rows, `0` sim-time resets.
- Truth callback rate: about `83 Hz` mean, with max frame period about
  `50 ms`.
- px4xplane version in X-Plane log: `v3.4.31`.
- Active aircraft and bridge config: `QuadTailsitter`.

## Parameter Caveat

The run did not use a clean airframe default load:

- ULog showed `SYS_AUTOCONFIG=0`.
- Several boot parameters did not match the v3.4.31 file on disk, including
  `MPC_TILTMAX_AIR=45`, `MPC_VEL_MANUAL=5`, `MC_YAWRATE_P=0.20`, and
  `MC_YAWRATE_D=0.04`.

Those values are useful evidence, but the next test must force a clean PX4
parameter load. Use one of:

- `make distclean`, then rebuild/run `xplane_qtailsitter`.
- Delete `build/px4_sitl_default/rootfs/parameters.bson` and
  `parameters_backup.bson`.
- Run `param reset_all` in PX4 shell, stop PX4, then restart.

Do not judge a new parameter file if the next ULog again shows stale/manual
values that differ from the sanity-check card.

## Go-To Finding

QGC sent `MAV_CMD_DO_REPOSITION` at about `t+43.2`, `t+80.3`, and `t+131.3 s`.
Each command had `param1=-1`, so QGC did not send a custom cruise speed. PX4
used the normal multicopter position-speed limits.

The speed changes are mostly expected point-reposition behavior:

- First Go-To: actual speed max about `4.5 m/s`.
- Second Go-To: actual speed max about `5.1 m/s`.
- Third Go-To: actual speed max about `3.8 m/s`.
- PX4 slowed the trajectory setpoint near the current target, then accelerated
  again after the next target was commanded.

This is not a bridge, GPS, or estimator problem. Go-To is a point-hold command,
not a blended multi-waypoint path. For a continuous path, use a mission or send
the next target before the aircraft reaches the current one.

The live values did confirm that the aircraft can use more tilt/yaw authority
than v3.4.31, so v3.4.32 promotes a moderated version of those live values.

## Transition Finding

The ULog contains six forward-transition attempts:

| Start | End | Duration | Outcome |
| --- | --- | --- | --- |
| `185.2 s` | `189.5 s` | `4.3 s` | quad-chute, large altitude loss/min-alt compatible |
| `220.2 s` | `225.4 s` | `5.2 s` | quad-chute, large altitude loss/min-alt compatible |
| `318.5 s` | `331.2 s` | `12.7 s` | quad-chute, transition did not complete |
| `396.1 s` | `412.3 s` | `16.2 s` | timeout-compatible |
| `490.3 s` | `503.6 s` | `13.3 s` | altitude-loss compatible |
| `556.3 s` | `570.2 s` | `13.9 s` | altitude-loss compatible |

Every attempt entered `TRANSITION_TO_FW` and returned to multicopter mode. None
reached fixed-wing state.

The primary blocker was airspeed gating:

- Airspeed params at boot: `FW_USE_AIRSPD=1`, `VT_ARSP_TRANS=15`.
- PX4 validated airspeed max during transitions: about `0.8-2.0 m/s`.
- Truth groundspeed max during transitions: about `35-76 m/s`.
- Truth true airspeed max in the run: about `76 m/s`.
- X-Plane indicated airspeed ranged from about `-134 kt` to `+8 kt`.

So PX4 was waiting for `15 m/s` calibrated airspeed while the source remained
finite and near zero. The aircraft then kept pitching/accelerating until
timeout, altitude loss, or attitude recovery triggered quad-chute.

## Bridge Finding

The bridge was not inventing the zero airspeed from PX4 side, but it was
hiding one useful diagnostic:

- X-Plane IAS can become negative during high-AoA/reverse-flow tailsitter
  motion.
- px4xplane v3.4.31 clamped negative IAS to zero before calculating
  `HIL_SENSOR.diff_pressure`.
- PX4's `differential_pressure` topic explicitly allows negative pressure.

v3.4.32 preserves signed IAS as signed differential pressure:

`q = 0.5 * rho * V * abs(V)`

This is a general sensor-contract cleanup, not a QuadTailsitter-only hack. It
does not fake transition airspeed from true-speed magnitude, and it does not
use `abs(IAS)`.

## Heading Finding

QGC's small vehicle icon/heading can look unstable during tailsitter
pitch-over. The ULog and truth attitude both show the aircraft moving near
`-90 deg` pitch with roll often near `180 deg`. Euler yaw/roll are
ill-conditioned near vertical attitude, and PX4 does not send the fixed-wing
tailsitter display offset until after transition completes.

This is mostly display/representation behavior. Judge the next transition by:

- `vtol_vehicle_status.vehicle_vtol_state`
- local altitude and velocity
- quaternion attitude and pitch/roll after transition should have stabilized
- actuator saturation and quad-chute reason

## Implemented v3.4.32 Changes

Bridge/replay:

- Preserve signed X-Plane IAS when generating `HIL_SENSOR.diff_pressure`.
- Update `tools/replay_truth_capture.py` to decode signed differential
  pressure the same way.
- Add a replay unit test for negative IAS.

QuadTailsitter params:

- Make the transition airspeed policy internally consistent:
  `FW_USE_AIRSPD=0`, `ASPD_DO_CHECKS=0`, `SYS_HAS_NUM_ASPD=0`,
  `VT_ARSP_BLEND=0`, `VT_ARSP_TRANS=0`.
- Slow first front-transition pitch-over:
  `VT_F_TRANS_DUR=6.0`, `VT_F_TRANS_THR=0.65`, `VT_TRANS_TIMEOUT=20`.
- Raise transition safety margins:
  `VT_FW_MIN_ALT=40`, `VT_QC_T_ALT_LOSS=35`.
- Promote moderated live MC values:
  `MPC_VEL_MANUAL=5.0`, `MPC_MAN_TILT_MAX=35`,
  `MPC_TILTMAX_AIR=40`, `MC_YAW_P=0.80`,
  `MC_YAWRATE_P=0.20`, `MC_YAWRATE_D=0.025`,
  `MC_YAWRATE_K=1.20`, `MC_YAWRATE_MAX=65`.

## Next Test

Run one clean v3.4.32 test:

1. Force a PX4 param reset with `make distclean` or by deleting
   `parameters.bson` and `parameters_backup.bson`.
2. Confirm the ULog sanity-check params match
   `docs/QUADTAILSITTER_XPLANE12_TEST.md`.
3. Take off, hold, command one `3 m/s` Go-To, one `4-5 m/s` Go-To, then one
   `40-50 m` Orbit.
4. If MC behavior is clean, climb to at least `80 m` AGL and command one
   forward transition into open space.
5. If it reaches FW state cleanly, keep it straight and shallow for `10-15 s`,
   then back-transition or RTL.

Acceptance:

- Go-To and Orbit remain controlled without sustained attitude error.
- Signed `differential_pressure` appears in ULog when X-Plane IAS is negative.
- Transition no longer waits forever on zero airspeed.
- If transition still fails, the reason is visible in altitude, attitude,
  timeout, or actuator evidence.

## References

- PX4 Tailsitter VTOL overview:
  https://docs.px4.io/v1.14/en/frames_vtol/tailsitter
- PX4 VTOL without airspeed sensor guidance:
  https://docs.px4.io/v1.12/en/config_vtol/vtol_without_airspeed_sensor
- PX4 VTOL transition tips:
  https://docs.px4.io/v1.13/en/config_vtol/vtol_quad_configuration
- PX4 DifferentialPressure message:
  https://docs.px4.io/main/en/msg_docs/DifferentialPressure
