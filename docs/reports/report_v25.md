# Report v25 - Alia Test7 Orbit Recovery

Date: 2026-05-18

Scope: analyze `/home/alireza/alia-test7.zip`, verify whether v3.4.12 params
were active, explain the takeoff/transition/orbit/throttle observations, and
prepare the v3.4.13 correction package.

## Evidence

Test7 used the intended v3.4.12 package and PX4 airframe:

- X-Plane loaded `px4xplane v3.4.12 (Alia FW Tuning)`.
- ULog had `SYS_AUTOSTART=5020`.
- ULog params included `NPFG_PERIOD=70`, `NPFG_DAMPING=0.85`,
  `WEIGHT_BASE=3120`, `WEIGHT_GROSS=3120`, `VT_ARSP_TRANS=46`,
  `VT_B_DEC_MSS=2.2`, and the v3.4.12 TECS values.
- XPlaneTruthCapture recorded `52,149` frames over `736.9 s`, zero dropped
  rows, and the aircraft mass was `3118.45 kg`.

No bridge sign/axis rewrite is indicated by this run. Baro and airspeed aid
sources were not rejected. The run did show a short vertical-acceleration
estimator flag during back-transition, so back-transition remains a watch item.

## Diagnosis

The v3.4.12 lateral tune went in the wrong direction. `NPFG_PERIOD=70` made the
loiter capture too lazy, but PX4 still reached the roll limit:

- FW loiter roll setpoint was at about `+-30 deg` for more than `90%` of FW
  loiter/RTL.
- `fixed_wing_lateral_guidance_status.signed_track_error` reached roughly
  `-1000 m` to `+900 m` during loiter.
- `adapted_period` stayed at `70 s`, so the long look-ahead was actually active.

The vertical and throttle problem is coupled to that lateral behavior:

- TECS throttle was above `0.999` for about `61%` of FW loiter and `98%` of FW
  RTL.
- Elevator output saturated high for about `38%` of FW loiter and `58%` of FW
  RTL.
- TruthCapture confirmed the pusher was actually at full throttle in FW and the
  aircraft still spent long periods in high bank.

The takeoff climb rate was not changed by the FW tuning. PX4 used
`MPC_TKO_SPEED=1.5`, which is the takeoff climb-rate parameter. The normal MC
auto climb limit `MPC_Z_V_AUTO_UP=3.0` is separate.

The "stopped near 60 m" observation is real in the ULog: the local MC setpoint
plateaued near `62.9 m` local, then an external Auto Loiter command at
`142.47 s` locked the current altitude near `97 m MSL`. For the next test, use
the QGC takeoff altitude and the ULog sanity values together; do not judge the
airframe file from the plugin reference copy.

Front-transition pitch behavior is explained by PX4 source: Standard VTOL ramps
pitch toward `FW_PSP_OFF`; there is no separate late pitch-up feature at the FW
switch. `FW_PSP_OFF` was `0`, while TruthCapture showed level FW pitch around
`3-4 deg`, so v3.4.13 adds a small trim offset.

## v3.4.13 Changes

- `MPC_TKO_SPEED=3.0`
- `FW_PSP_OFF=3.0`
- `FW_P_LIM_MAX=18.0`
- `FW_R_LIM=22.0`
- `FW_R_RMAX=18.0`
- `FW_R_TC=0.7`
- `FW_T_RLL2THR=12.0`
- `NPFG_PERIOD=35.0`
- `NPFG_DAMPING=0.80`
- `NPFG_ROLL_TC=0.7`
- `NAV_LOITER_RAD=1200.0`
- `RTL_LOITER_RAD=1200.0`

Bridge/plugin cleanup:

- Normal compact bridge diagnostics now default to off.
- Magnetic-field update messages are gated behind verbose logging.
- Packaged `px4_airframes` are documented as reference/install copies only.
  PX4 SITL reads the airframe file from the PX4 repository branch.

## Next Test Focus

Use a PX4 `distclean` once after pulling/replacing the airframe. Verify in the
ULog that the v3.4.13 sanity values are active before judging the result.

Watch:

- QGC takeoff climb near `3 m/s` and target altitude behavior.
- Front transition pitch should ramp toward about `3 deg` before FW takeover.
- FW loiter should stop the lazy capture / hard bank reversal pattern.
- Pusher throttle should spend less time pegged at `1.0`.
- Elevator saturation and roll setpoint saturation should drop substantially.
- Back-transition should complete without vertical-acceleration estimator flags.

## References

- PX4 fixed-wing altitude/position tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 Standard VTOL transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 takeoff climb parameter definition:
  `/home/alireza/PX4-Autopilot-Me/src/modules/mc_pos_control/multicopter_takeoff_land_params.c`
- PX4 Standard VTOL transition pitch source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/vtol_att_control/standard.cpp`
- PX4 NPFG and FW lateral control source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/fw_mode_manager/fw_mode_manager_params.c`
