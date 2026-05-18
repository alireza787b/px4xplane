# Report v24 - Alia Test6 Fixed-Wing Tuning

Date: 2026-05-18

Scope: analyze `/home/alireza/alia-test6.zip`, explain remaining fixed-wing
vertical/orbit/back-transition issues, and prepare the v3.4.12 tuning package.

## Evidence

The run is valid evidence:

- X-Plane loaded `px4xplane v3.4.11 (Alia Crash Recovery)`.
- XPlaneTruthCapture v0.1.7 recorded `36,294` rows with zero dropped rows.
- PX4 launched `SYS_AUTOSTART=5020`; the ULog had no dropouts.
- VTOL state reached fixed-wing at about `138.8 s`, with no quad-chute.
- Baro source stayed on device `6620172`; estimator health/fault/timeout flags
  stayed zero.

Front transition was the best run so far, but fixed-wing control was not yet
demo quality:

- TECS throttle setpoint was above `0.99` for about `91%` of fixed-wing flight.
- TruthCapture confirmed the pusher throttle really was `1.0` in X-Plane, so
  this is not a QGC display or bridge throttle mapping bug.
- Fixed-wing altitude oscillated about `16-24 m` peak-to-peak, with vertical
  velocity commonly near `3-5 m/s`.
- FW orbit guidance spent about `90%` of FW flight with `|roll setpoint| > 25 deg`
  and flipped roll sign every roughly `17-19 s`.
- Back-transition started at about `54.2 m/s`, completed around `10.6 m/s`, and
  averaged about `2.27 m/s^2` deceleration, lower than the configured
  `VT_B_DEC_MSS=3.0`.

TruthCapture also showed X-Plane aircraft mass around `3118 kg`, while PX4 was
configured with `WEIGHT_GROSS=3500`. That weight mismatch inflated the effective
FW/transition airspeed targets and contributed to TECS chasing excess speed and
energy.

## Diagnosis

The remaining problem is coupled fixed-wing tuning, not a sensor-contract
failure.

TECS was saturated because it was trying to recover altitude while flying:

- a weight-scaled true airspeed target near `55.6 m/s`,
- repeated high-bank orbit corrections,
- and a pitch/elevator loop that spent significant time at its positive command
  limit.

PX4's fixed-wing tuning guide says TECS tuning mainly depends on correct
airframe limits, including trim throttle, pitch/climb limits, and measured climb
and sink performance. The parameter reference also states that `WEIGHT_BASE` and
`WEIGHT_GROSS` affect trim-throttle and minimum-airspeed compensation. In this
run those values did not match the X-Plane aircraft mass.

The orbit problem is NPFG/roll-command aggressiveness. `NPFG_PERIOD=45` was not
smooth enough for this large aircraft and the roll command repeatedly saturated.
PX4 guidance says vehicles with slow roll dynamics should increase the NPFG
period; the v3.4.12 slice therefore smooths NPFG and roll response instead of
changing sensor signs or rewriting the bridge.

The back-transition overshoot follows PX4's documented prediction model:
`VT_B_DEC_MSS` is used to decide where to start back-transition. The measured
deceleration was about `2.27 m/s^2`, so the previous `3.0` value made PX4 expect
more braking than the aircraft delivered and start too late.

## v3.4.12 Changes

### px4xplane

- Bumped version to `3.4.12`, build `020`.
- Fixed the `No autoPropBrakes...` X-Plane log line so it ends cleanly before
  the motor-brake summary.
- Added an always-run packaged-assets refresh target so local builds do not keep
  stale `config.ini`, docs, or PX4 airframe files when only those assets change.

### Alia PX4 Airframe

- Matched the PX4 FW performance mass model to the X-Plane aircraft:
  - `WEIGHT_BASE=3120`
  - `WEIGHT_GROSS=3120`
- Smoothed fixed-wing energy control:
  - `FW_T_ALT_TC=6.0`
  - `FW_T_STE_R_TC=2.0`
  - `FW_T_SINK_MIN=1.5`
  - `FW_T_SINK_MAX=3.0`
  - `FW_T_CLMB_MAX=3.0`
  - `FW_T_VERT_ACC=3.0`
  - `FW_T_THR_DAMPING=0.25`
  - `FW_T_PTCH_DAMP=0.25`
  - `FW_T_I_GAIN_PIT=0.20`
  - `FW_T_CLMB_R_SP=2.0`
  - `FW_T_SINK_R_SP=1.5`
- Smoothed lateral orbit behavior:
  - `NPFG_PERIOD=70.0`
  - `NPFG_DAMPING=0.85`
  - `NPFG_ROLL_TC=1.0`
  - `NPFG_SW_DST_MLT=0.7`
  - `FW_R_RMAX=12.0`
  - `FW_R_TC=0.9`
- Adjusted back-transition prediction/control to measured behavior:
  - `VT_B_DEC_MSS=2.2`
  - `VT_B_DEC_I=0.25`

## Prop-Brake Status

The unsafe prop-brake regression is solved in the generic bridge policy:
braking is opt-in per motor list, applies only after all configured motors have
stayed low, releases all brakes if any configured motor requests recovery, uses
dwell/airspeed gates, and does not use X-Plane seizure failures unless an
airframe explicitly enables that experimental option.

Alia windmilling itself is not declared solved yet. For Alia, auto prop braking
remains disabled because test5 proved lift props can be needed again during
transition/recovery. A future safe Alia solution should be mode-aware or
explicitly commanded, then validated with replay and SITL before re-enabling.

## Next Test

Use the v3.4.12 package with a PX4 `distclean` once after replacing the airframe
file.

Watch specifically for:

- front transition still reaches fixed-wing cleanly,
- fixed-wing throttle no longer sits at 100% for nearly the whole orbit,
- FW roll setpoint spends much less time saturated and stops reversing every
  ~17-19 s,
- vertical speed peaks fall below the test6 `3-5 m/s` band,
- back-transition starts earlier and climbs less while still slowing below
  multicopter cruise speed,
- no estimator/baro/airspeed warnings.

## References

- PX4 fixed-wing altitude/position tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 FW performance/weight parameter definitions:
  `/home/alireza/PX4-Autopilot-Me/src/lib/fw_performance_model/performance_model_params.c`
- PX4 NPFG/roll parameter definitions:
  `/home/alireza/PX4-Autopilot-Me/src/modules/fw_mode_manager/fw_mode_manager_params.c`
- BETA ALIA high-level aircraft context:
  https://www.beta.team/aircraft/
