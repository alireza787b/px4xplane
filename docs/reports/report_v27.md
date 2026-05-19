# Report v27 - Alia Bank Recovery and Ehang Nav Tuning

Date: 2026-05-19

Scope: analyze `/home/alireza/evtol1.zip`, identify why v3.4.14 regressed Alia,
triage the first Ehang 184 flight, update both plugin-reference and PX4-fork
airframes, and prepare the next validation package.

## Evidence Reviewed

The zip contained three PX4 logs and two TruthCapture runs:

- `03_52_57.ulg`: short low-FPS Alia startup/preflight warning run.
- `03_54_36.ulg`: v3.4.14 Alia flight.
- `04_05_42.ulg`: v3.4.14 Ehang 184 flight.
- Truth run `20260519-035119Z`: `40,004` frames, `666 s`, zero dropped rows,
  no sim-time reset, p50 frame period about `14.5 ms`, with early low-FPS
  samples down near `20-40 Hz`.
- Truth run `20260519-040502Z`: `47,950` frames, `538 s`, zero dropped rows,
  no sim-time reset, mean callback rate about `89 Hz`.

## Alia Root Cause

The v3.4.14 regression was not a new sensor-sign or MAVLink-unit failure. It was
the fixed-wing bank envelope change.

Comparison against the accepted v3.4.13 Alia run:

- v3.4.13 used `FW_R_LIM=22`. Actual fixed-wing bank stayed below about
  `28 deg`; the fraction above `34 deg` was `0`.
- v3.4.14 used `FW_R_LIM=35`. Actual fixed-wing bank reached about
  `44 deg`, and the aircraft spent about `45%` of fixed-wing time above
  `34 deg`.
- v3.4.14 TECS commanded max throttle for about `91%` of fixed-wing time.
- Elevator output saturated high for about `56%` of fixed-wing time.
- During high-bank segments, the aircraft commonly sank several meters per
  second even while TECS asked for more pitch and throttle.

That matches the visual report: the aircraft captured late, stayed near max
bank, lost altitude in the turn, crossed the path, then repeated the same
high-bank correction in the opposite direction. Raising the bank limit to satisfy
PX4 parameter metadata made this specific X-Plane Alia model fly worse.

v3.4.15 therefore restores `FW_R_LIM=22` and increases fixed-wing loiter/RTL
radii to `2000 m`. This intentionally prioritizes the ULog-proven demo envelope.
The upstream PX4 metadata conflict should be handled later as a review item,
not solved by forcing this X-Plane model back into the bad `35 deg` behavior.

## Alia Windmilling

v3.4.14 had Alia `autoPropBrakes` disabled. TruthCapture showed that in
fixed-wing flight the lift motors were commanded near zero, but the lift props
still had measurable rotation and negative torque. That is consistent with
windmilling drag and helps explain why the pusher spent so much time at max
throttle.

The old unsafe failure-style brake behavior remains disabled. v3.4.15 uses the
generic opt-in brake policy already in the bridge:

- apply only to motors `0-3`
- apply only after all configured motor commands stay below `0.01`
- require `4 s` dwell time
- require true airspeed above `50 m/s`
- release immediately if any configured motor command rises above `0.05`
- keep `autoPropBrakeUseFailure=false`

This is still reusable and airframe-configurable. Nothing is hard-coded in C++
for Alia.

## Low-FPS Startup Warning

The short low-FPS Alia log showed repeated preflight `High Accelerometer Bias`,
`height estimate not stable`, and `vertical velocity unstable` warnings. It did
not show MAVLink dropouts. The early sensor cadence was much lower than the good
run, but the raw TruthCapture gravity values were normal.

The main fix in this slice is to stop seeding fresh SITL runs with stale accel
offsets. v3.4.15 updates Alia `CAL_ACC0_*OFF` from the latest clean Alia log:

- `CAL_ACC0_XOFF=0.205`
- `CAL_ACC0_YOFF=-0.160`
- `CAL_ACC0_ZOFF=-0.714`

This does not replace the wider frame-rate robustness work. It removes a known
bad initial condition so lower-FPS tests are not contaminated by stale
calibration before we judge the bridge timing.

## Ehang Findings

The Ehang ULog used `SYS_AUTOSTART=5010` and had no major in-flight estimator
messages. The terminal warning:

`LNDMC_Z_VEL_MAX > MPC_LAND_CRWL or MPC_LAND_SPEED`

is explained by PX4 land-detector logic: with `MPC_LAND_CRWL=0.25`, the
land-detector max vertical velocity must be lower than `0.25 / 1.2`, about
`0.208 m/s`. v3.4.15 sets `LNDMC_Z_VEL_MAX=0.20` for both Alia and Ehang.

The QGC "Go to location" commands were accepted, but PX4 stayed in Position
mode and the local position setpoint did not move. This matches PX4 commander
and navigator behavior for reposition commands that do not switch into an auto
mode that consumes the reposition setpoint. It is a command/mode behavior issue,
not evidence that the Ehang motors or px4xplane actuator mapping are broken.

The Ehang orbit was too tight for the first validation. The log showed QGC
commanded a `30 m` orbit. PX4's Orbit mode can accept that, but PX4 documents
that orbit behavior is constrained by commanded radius, `MPC_XY_VEL_MAX`, and a
centripetal acceleration limit. For a large passenger-style multicopter, the
next test should command `80-100 m` orbit radius.

v3.4.15 Ehang changes:

- add Ehang-specific `CAL_ACC0` seed offsets from the clean Ehang log
- `MPC_TKO_SPEED=2.0`
- `MPC_XY_CRUISE=6`
- `MPC_XY_VEL_MAX=8`
- `MPC_VEL_MANUAL=6`
- `MPC_Z_V_AUTO_DN=1.2`
- `NAV_ACC_RAD=12`
- `LNDMC_Z_VEL_MAX=0.20`

## Files Changed

px4xplane:

- `config/config.ini`
- `config/px4_params/5020_xplane_alia250`
- `config/px4_params/5010_xplane_ehang184`
- `docs/ALIA_XPLANE12_TEST.md`
- `docs/EHANG184_XPLANE12_TEST.md`
- `docs/index.md`
- `CHANGELOG.md`
- `JOURNEY_LOG.md`

PX4 fork:

- `ROMFS/px4fmu_common/init.d-posix/airframes/5020_xplane_alia250`
- `ROMFS/px4fmu_common/init.d-posix/airframes/5010_xplane_ehang184`

## Next Test Instructions

Use the v3.4.15 package. Run `distclean` once after replacing the PX4 airframe
files or pulling the PX4 fork branch. Confirm:

- X-Plane log says `px4xplane: Version: v3.4.15`.
- Alia PX4 log says `SYS_AUTOSTART=5020`, `FW_R_LIM=22`,
  `NAV_LOITER_RAD=2000`, `RTL_LOITER_RAD=2000`.
- Alia X-Plane log says `Motor brakes configured for motors: 00001111`.
- Ehang PX4 log says `SYS_AUTOSTART=5010`, `MPC_XY_CRUISE=6`,
  `MPC_XY_VEL_MAX=8`, `MPC_VEL_MANUAL=6`, `MPC_TKO_SPEED=2`,
  `NAV_ACC_RAD=12`.

For Alia, repeat the accepted workflow: takeoff to about `100 m`, forward
transition, large circle/RTL, back-transition, land, wait `10-15 s`, then stop
PX4.

For Ehang, use takeoff, hover, yaw, translation, climb/descent, Orbit with
`80-100 m` radius, RTL, and land. Treat QGC "Go to location" as a separate
mode-command check unless PX4 switches to an auto mode that follows the
reposition setpoint.

## References

- PX4 fixed-wing altitude/position tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 multicopter trajectory tuning:
  https://docs.px4.io/main/en/config_mc/mc_trajectory_tuning.html
- PX4 multicopter Orbit mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit
- PX4 QuadPlane VTOL configuration and tuning:
  https://docs.px4.io/v1.13/en/config_vtol/vtol_quad_configuration.html
- PX4 land-detector source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/land_detector/MulticopterLandDetector.cpp`
- PX4 reposition handling source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/commander/Commander.cpp`
- PX4 Orbit flight-task source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/flight_mode_manager/tasks/Orbit/FlightTaskOrbit.cpp`
