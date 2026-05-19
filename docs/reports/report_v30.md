# Report v30 - Alia Brake Recovery and Ehang Orbit Damping

Date: 2026-05-19

Scope: review `/home/alireza/evtol4.zip`, identify why Alia crashed after
fixed-wing transition, explain the persistent Ehang orbit yo-yo, and prepare the
v3.4.18 package without adding another speculative bridge rewrite.

## Evidence Reviewed

The archive contained:

- `11_15_34.ulg`: Alia, `SYS_AUTOSTART=5020`, mission-mode flight.
- `11_21_44.ulg`: Ehang 184, `SYS_AUTOSTART=5010`.
- TruthCapture `20260519-111501Z`: Alia `ALIA-250.acf`, `23,241` frames, zero
  dropped rows.
- TruthCapture `20260519-112111Z`: Ehang `quadricopter.acf`, `30,534` frames,
  zero dropped rows.
- X-Plane log showing px4xplane `v3.4.17`.

## Alia Finding

The Alia failure was not missing pusher throttle, not stale validated airspeed,
and not a broad sensor-contract regression.

At the fixed-wing switch:

- `airspeed_validated.calibrated_airspeed_m_s` was about `50.0 m/s`.
- raw `airspeed.indicated_airspeed_m_s` also read about `50.0 m/s`.
- PX4 commanded the pusher at full output (`actuator_motors[4]=1.0`,
  `output[8]=2000`).
- TruthCapture showed X-Plane engine index 4 near full throttle and producing
  about `4 kN` thrust.

The first seconds after FW switch looked like the accepted v3.4.13 baseline:
both runs lost roughly `20 m` during the first few seconds, then began to
recover. In `evtol4`, the recovery ended when the lift-prop brake window became
active. After that, TECS commanded full throttle and maximum nose-up pitch, the
elevator saturated, actual pitch went negative, and the aircraft sank into
quad-chute.

Key comparison:

- v3.4.13 accepted run: no default lift-prop brakes, recovered after initial FW
  sink and completed the workflow.
- v3.4.17 `evtol4`: brake-enabled run recovered initially, then lift-prop
  braking removed the remaining margin and the aircraft crashed.

## Alia Decision

v3.4.18 restores the accepted no-brake Alia baseline:

- `autoPropBrakes =` empty by default.
- `VT_ARSP_TRANS=46.0`, matching the accepted no-brake handoff baseline.
- `FW_PSP_OFF=3.0`, matching the accepted no-brake trim baseline.
- `ASPD_DO_CHECKS=1` remains from v3.4.17 because it fixed validated airspeed in
  the virtual-pitot SITL workflow.
- `FW_R_LIM=22`, `NAV_LOITER_RAD=2000`, and `RTL_LOITER_RAD=2000` remain.

The generic brake code is kept in the bridge for controlled A/B tests, including
`feather`, `hard_lock`, and `prop_separate`. It is not enabled for the main
Alia validation package.

## Ehang Finding

The Ehang orbit issue is not a missing roll channel. In `evtol4`, QGC commanded
an Orbit with:

- radius about `97.7 m`
- velocity `NaN`, so PX4 used its default slow orbit behavior
- yaw behavior `0`, nose/front toward the circle center

PX4 Orbit mode first captures the closest point on the commanded circle, then
flies a slow orbit. In this log, the radial correction dominated the commanded
motion. The radius error repeatedly crossed from inside to outside the circle,
and the commanded radial velocity flipped between roughly `-6.5` and `+6.5 m/s`.

The bad channel was pitch response:

- pitch setpoint repeatedly hit about `+-30 deg`
- actual pitch lagged badly, roughly `-19..+18 deg`
- roll setpoint stayed small and yaw tracking error was low
- motor outputs were not saturated high

This points to underdamped MC tuning. The airframe had `MC_ROLL_P=0.3` and
`MC_PITCH_P=0.3`, far below the PX4 default attitude P gain of `4.0`, while
`MPC_XY_VEL_D_ACC=1.5` was far above the PX4 default `0.2`. PX4 parameter docs
warn that too much horizontal velocity D gain can bring oscillations back.

## Ehang Decision

v3.4.18 changes Ehang only where the log showed a problem:

- `MC_ROLL_P=3.0`
- `MC_PITCH_P=3.0`
- `MC_YAW_P=1.2`
- `MPC_XY_P=0.30`
- `MPC_XY_VEL_P_ACC=1.4`
- `MPC_XY_VEL_D_ACC=0.25`
- `MPC_XY_CRUISE=4.0`
- `MPC_XY_VEL_MAX=5.0`
- `MPC_VEL_MANUAL=4.0`
- `MPC_ACC_HOR=2.0`
- `MPC_ACC_HOR_MAX=2.0`
- `MPC_TILTMAX_AIR=25.0`

The acceleration values are kept at PX4-valid minimums instead of using invalid
sub-minimum values.

## Files Changed

px4xplane:

- `CMakeLists.txt`
- `include/VersionInfo.h`
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

Use v3.4.18 and run `distclean` once before judging the package.

For Alia:

- confirm `SYS_AUTOSTART=5020`
- confirm X-Plane log says `px4xplane: Version: v3.4.18`
- confirm X-Plane log says no `autoPropBrakes` are configured for `Alia250`
- confirm `ASPD_DO_CHECKS=1`
- confirm `VT_ARSP_TRANS=46.0`, `FW_PSP_OFF=3.0`, `FW_R_LIM=22.0`
- run the same mission-mode and manual RTL/loiter workflow if possible
- keep TruthCapture recording through FW switch and landing

For Ehang:

- confirm `SYS_AUTOSTART=5010`
- confirm `MC_ROLL_P=3.0`, `MC_PITCH_P=3.0`, `MPC_XY_VEL_D_ACC=0.25`
- use Orbit radius `80-100 m`
- judge the orbit after capture, not during the initial fly-to-circle segment
- look for reduced pitch setpoint flipping and smaller radius hunting

## References

- PX4 Standard VTOL / QuadPlane transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration
- PX4 fixed-wing altitude/position tuning:
  https://docs.px4.io/v1.15/en/config_fw/position_tuning_guide_fixedwing.html
- PX4 multicopter Orbit mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit
- PX4 parameter reference for `MPC_XY_VEL_D_ACC` and related MC position gains:
  https://docs.px4.io/v1.15/en/advanced_config/parameter_reference
- PX4 local source used in this review:
  `/home/alireza/PX4-Autopilot-Me/src/modules/flight_mode_manager/tasks/Orbit/FlightTaskOrbit.cpp`
- PX4 local source used for MC attitude defaults:
  `/home/alireza/PX4-Autopilot-Me/src/modules/mc_att_control/mc_att_control_params.c`
