# Report v60 - qtail24 Back-Transition Dive and v3.4.48 Fixes

Date: 2026-05-26
Package target: `v3.4.48`

## Executive Summary

`qtail24.zip` was a real improvement over qtail23. X-Plane loaded px4xplane
`v3.4.47`, PX4 loaded the intended `5021_xplane_qtailsitter` defaults, the
body-axis `-Z` pitot was active, and TruthCapture recorded `34,537` frames with
`0` dropped rows and `0` sim-time resets. Front transition and fixed-wing path
tracking were much better.

The remaining issues are now narrower:

- FW cruise still has a low-frequency yaw/roll wobble. It is guidance/control
  induced, not a thrust shortage.
- Mission VTOL Land still starts a tailsitter back-transition at too much
  energy for the current pitch response. The aircraft did not rotate toward
  hover fast enough, accelerated downward, then quad-chuted on minimum altitude.

No bridge sensor-contract or ACF physics change is included in this slice.

## qtail24 Evidence

Timing and setup:

- TruthCapture: `34,537` rows, `436.6 s` sim duration, about `79 Hz` mean
  callback rate, no pause rows, no sim-time resets, maximum frame period about
  `50 ms`.
- X-Plane log: `px4xplane v3.4.47`, `Config Name: QuadTailsitter`,
  `Airspeed source=body_axis pitotAxisBody=-Z`.
- ULog parameter check: all checked qtail defaults matched the current
  airframe file, except `SYS_AUTOSTART` is expected to come from PX4 startup.

Flight phases:

- MC climb held about `2 m/s` target behavior and stayed stable.
- Front transition ran from about `154.1 s` to `161.7 s`.
- FW mission cruise held about `100 m AGL`, TAS around `32-33 m/s`, and TECS
  throttle around `0.12` median.
- Mission back-transition started around `311.9 s` at about `100 m AGL`,
  `31.6 m/s` ground speed, and `31 m/s` TAS.
- By quad-chute, altitude had fallen to about `25-30 m AGL`, vertical speed
  down reached about `27 m/s`, and TAS increased to about `56 m/s`.

During back-transition PX4 was not idling the aircraft: MC thrust went high and
the motors were near saturation. The problem was that thrust was being applied
while the vehicle was still nearly in fixed-wing attitude, so the aircraft kept
accelerating and could not build a safe hover recovery before the altitude gate.

## Aerodynamic Sizing

The current aircraft sizing is plausible for this stage:

- Loaded truth mass: `5.44 kg`.
- Wing area estimate: about `0.509 m^2`.
- Loaded wing loading: about `105 N/m^2` (`10.7 kg/m^2`).
- Estimated stall speed at `CLmax=1.2`: about `12 m/s`.
- At `32 m/s`, required lift coefficient is only about `0.17`.
- At the new `29 m/s` target, required lift coefficient is about `0.20`.

So the wing is not obviously too small or too large. qtail24 cruise at
`32 m/s` was smooth enough to prove the model can fly, but it leaves too much
back-transition energy. v3.4.48 therefore moves the trim target to `29 m/s`,
still well above stall but less punishing during recovery.

The prop/weight combination also looks usable. Hover uses about `0.22` motor
mean in the ULog, so static thrust margin exists. The failure appeared at high
advance ratio during a dive, where the vehicle was still not pitched into hover.
That is a transition-control problem first; changing motors/wing/weight now
would hide the evidence instead of fixing the root cause.

## v3.4.48 Changes

FW speed and energy:

- `FW_AIRSPD_MIN/TRIM/MAX`: `22/32/45 -> 20/29/40`
- `FW_THR_TRIM`: `0.20 -> 0.16`
- `VT_ARSP_BLEND/TRANS`: `20/24 -> 18/23`

FW wobble damping:

- `FW_R_TC`: `1.1 -> 1.3`
- `FW_R_LIM`: `20 -> 17`
- `FW_RR_P/FF`: `0.14/0.08 -> 0.12/0.06`
- `FW_YR_P`: `0.05 -> 0.02`
- `VT_FW_DIFTHR_S_R/P/Y`: `0.70/0.85/0.70 -> 0.60/0.90/0.45`
- `NPFG_PERIOD`: `40 -> 48`
- `NPFG_ROLL_TC`: `1.6 -> 2.0`
- `NAV_ACC_RAD`: `80 -> 90`
- `NAV_LOITER_RAD/RTL_LOITER_RAD`: `900 -> 1100`

Back-transition:

- `VT_B_TRANS_DUR`: `6.0 -> 4.0`
- `VT_B_DEC_MSS`: `1.6 -> 1.1`
- `VT_B_DEC_I`: `0.10 -> 0.12`
- `VT_B_TRANS_RAMP`: `3.0 -> 1.5`
- `MC_PITCH_P`: `0.50 -> 0.58`
- `MC_PITCHRATE_P/D/K/MAX`: `0.10/0.0010/0.85/80 -> 0.115/0.0012/0.92/110`

Why this mix:

- PX4’s tailsitter code rotates the back-transition attitude setpoint at
  `90 deg / VT_B_TRANS_DUR`. qtail24 proved `6 s` was still too slow.
- PX4’s back-transition guide says high `VT_B_TRANS_DUR` gives more glide time
  but only altitude, not position, is controlled during this phase. That matches
  the qtail24 dive evidence.
- PX4’s fixed-wing tuning guide says vehicles with slow roll dynamics should
  increase `NPFG_PERIOD`. qtail24 roll setpoint spent too much time near
  `FW_R_LIM`, so v3.4.48 softens guidance instead of adding roll authority.

## What To Test Next

Use v3.4.48 with a PX4 parameter reset:

1. Install the v3.4.48 package and pull the updated PX4 branch.
2. Run `make distclean` before judging the qtail run.
3. Confirm X-Plane log shows `v3.4.48`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
4. Confirm the ULog has `FW_AIRSPD_TRIM=29`, `FW_R_LIM=17`,
   `NPFG_PERIOD=48`, and `VT_B_TRANS_DUR=4`.
5. Take off, do one MC Go-To, climb to at least `150 m AGL`, transition to FW,
   and fly one large-radius leg/loiter.
6. Do one manual back-transition at high altitude before any VTOL Land mission.
7. If manual back-transition is clean, try VTOL Land with a high approach
   altitude. Do not use a `50 m` VTOL Land approach yet.

Acceptance for this slice:

- FW TAS should stabilize near `29-31 m/s`.
- FW yaw/roll wobble should be visibly smaller than qtail24, with no repeated
  roll-limit bang-bang behavior.
- Manual back-transition should not dive or quad-chute.
- VTOL Land should only be judged after the manual high-margin gate passes.

## References

- PX4 Tailsitter VTOL overview:
  https://docs.px4.io/v1.16/en/frames_vtol/tailsitter
- PX4 fixed-wing TECS/NPFG tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 tailsitter implementation reviewed locally:
  `src/modules/vtol_att_control/tailsitter.cpp`
