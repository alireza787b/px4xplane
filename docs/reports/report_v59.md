# Report v59 - qtail23 Mission Recovery Root Cause and v3.4.47 Fixes

Date: 2026-05-26
Package target: `v3.4.47`

## Executive Summary

`qtail23.zip` was not a stale-parameter or bridge-sensor regression. PX4 loaded
the intended v3.4.46 defaults, X-Plane loaded px4xplane v3.4.46, TruthCapture
recorded `33,891` frames with `0` dropped rows and `0` sim-time resets, and the
body-axis `-Z` pitot stayed positive in fixed-wing mode.

The failure was caused by fixed-wing energy and lateral-control tuning:

- The MC climb was intentionally limited by `MPC_Z_V_AUTO_UP=1.5`; actual climb
  averaged about `1.3 m/s`.
- The mission altitude frame was correct. PX4 commanded about `100 m AGL`, but
  after transition the aircraft ballooned to about `193 m` because it was flying
  around `38-42 m/s` while TECS targeted `24 m/s`.
- QGC throttle near zero was not proof that X-Plane motors were off. In this
  control-surface-less tailsitter, differential-thrust attitude control still
  drives the four rotors even when TECS longitudinal thrust is near zero.
- The low-frequency bank/yaw oscillation was severe. Roll setpoints saturated
  around `FW_R_LIM=32 deg`; truth sideslip reached roughly `+/-18 deg`.
- `VT_B_TRANS_DUR=14 s` was wrong for this PX4 tailsitter. PX4 rotates the
  tailsitter attitude by `90 deg / VT_B_TRANS_DUR`; qtail23 stayed nose-down too
  long, entered back-transition around `37 m/s` and `98 m AGL`, lost about
  `69 m` before quad-chute, then failed near `VT_FW_MIN_ALT=30`.

## Evidence Reviewed

From `/home/alireza/qtail23.zip`:

- TruthCapture: `33,891` frames, `411.9 s` host duration, about `82 Hz` mean
  callback rate, no dropped rows, no sim-time resets.
- X-Plane log: `px4xplane: Version: v3.4.46`, `Config Name: QuadTailsitter`,
  `Airspeed source=body_axis pitotAxisBody=-Z`, no sensor-contract warnings.
- ULog parameter check: current v3.4.46 qtail defaults matched the run.
- Transition timeline:
  - `238.81 s`: transition to FW started.
  - `246.11 s`: FW mode active.
  - `373.38 s`: transition back to MC started.
  - `380.02 s`: fixed-wing system failure after quad-chute.
- FW balloon window:
  - TAS median about `39.1 m/s`, p95 about `42.3 m/s`.
  - TECS TAS setpoint about `24.2 m/s`.
  - TECS throttle median `0.0`, but motor mean median about `0.34` because of
    differential-thrust attitude control.
- Back-transition/failure window:
  - Altitude fell from about `98 m` to about `29 m` in `6.6 s`.
  - Downward speed reached about `33 m/s`.
  - Motors saturated high, so this was not an installed-thrust shortage.

## Aircraft Sizing Decision

Do not change the ACF physics in this slice.

The loaded vehicle was about `5.44 kg`, wing area about `0.509 m^2`, wing
loading about `105 N/m^2`, and estimated stall about `11-12 m/s` at
`CLmax=1.2`. At the observed `38 m/s` FW speed the required lift coefficient is
only about `0.12`; at the new `32 m/s` trim target it is about `0.17`. The model
is over-energy because the FW controller is unstable and because rotor
differential thrust creates residual propulsive energy, not because the wing,
mass, CG, or prop size is obviously invalid.

The ACF watch item is drag/idle propulsive energy. If a future straight,
wings-level FW run still needs near-zero TECS throttle after lateral control is
stable, then add calibrated drag or revisit prop/idle modeling. qtail23 was too
oscillatory to justify that aircraft-physics change.

## v3.4.47 Changes

MC climb:

- `MPC_Z_V_AUTO_UP`: `1.5 -> 2.0`
- `MPC_Z_VEL_MAX_UP`: `1.8 -> 2.4`
- `MPC_ACC_UP_MAX`: `1.6 -> 2.2`
- `MPC_TKO_SPEED`: `1.0 -> 1.2`

FW energy:

- `FW_AIRSPD_MIN/TRIM/MAX`: `18/24/34 -> 22/32/45`
- `FW_THR_TRIM/MAX`: `0.16/0.40 -> 0.20/0.50`
- `FW_T_SPDWEIGHT`: `1.5 -> 0.8`
- `FW_T_CLMB_R_SP=2.0`, `FW_T_SINK_R_SP=2.0`
- `FW_PSP_OFF`: `4.0 -> 2.0`

FW lateral/differential thrust:

- `FW_R_TC`: `0.6 -> 1.1`
- `FW_R_LIM`: `32 -> 20`
- `FW_RR_P/I/FF`: `0.28/0.08/0.18 -> 0.14/0.04/0.08`
- `FW_YR_P`: `0.10 -> 0.05`
- `VT_FW_DIFTHR_S_R/P/Y`: `1.00/1.00/1.00 -> 0.70/0.85/0.70`

Transition and navigation:

- `VT_ARSP_BLEND/TRANS`: `15/21 -> 20/24`
- `VT_F_TRANS_THR`: `0.43 -> 0.36`
- `VT_B_TRANS_DUR`: `14.0 -> 6.0`
- `VT_B_DEC_MSS`: `0.8 -> 1.6`
- `VT_B_DEC_I`: `0.05 -> 0.10`
- `VT_B_TRANS_RAMP`: `10.0 -> 3.0`
- `NPFG_PERIOD/DAMPING/ROLL_TC`: `26/0.90/1.2 -> 40/1.00/1.6`
- `NPFG_SW_DST_MLT=0.70`
- `NAV_ACC_RAD=80`, `NAV_FW_ALT_RAD=30`
- `NAV_LOITER_RAD/RTL_LOITER_RAD`: `400/400 -> 900/900`

## Why These Fixes

PX4’s fixed-wing tuning guide says TECS tuning should follow a stable attitude
controller. qtail23 did not have stable attitude control, so v3.4.47 first
reduces FW roll demand, reduces roll-rate authority, and softens
differential-thrust scaling.

The PX4 parameter reference defines `FW_T_SPDWEIGHT` as the pitch-control
weighting between height and speed. qtail23 was overspeeding and ballooning
while TECS targeted `24 m/s`, so v3.4.47 moves trim speed closer to the observed
aircraft envelope and biases pitch back toward altitude.

The official quadtailsitter SITL airframes use `VT_B_TRANS_DUR=5`. In this PX4
tailsitter implementation, `VT_B_TRANS_DUR` directly determines pitch-over
schedule during back-transition; therefore v3.4.46’s `14 s` duration was too
slow for the available altitude. v3.4.47 uses `6 s` as a controlled recovery
value, not the Alia-style long ramp.

## Next Test

Use v3.4.47 with a clean PX4 parameter load:

1. Pull `PX4-Autopilot-Me` branch `px4xplane-sitl`.
2. Run `make distclean` before launching `xplane_qtailsitter`.
3. Install the v3.4.47 px4xplane package and packaged `QuadTailsitter`
   aircraft.
4. Confirm X-Plane log shows `v3.4.47`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
5. Take off, do one MC Go-To, then climb to at least `150 m AGL`.
6. Do one forward transition and one straight FW leg or large-radius loiter.
7. Do one manual back-transition while high before trying any VTOL Land item.

Acceptance for this slice:

- MC climb is near `1.8-2.0 m/s`.
- FW TAS stabilizes closer to the `32 m/s` trim target and does not balloon
  more than about `30 m`.
- Roll no longer sits at the limit or cycles with `25-30 deg` bang-bang motion.
- Manual back-transition does not lose more than about `40 m` and does not
  quad-chute.
- Do not use a `50 m` VTOL Land approach altitude until manual back-transition
  passes.

## References

- PX4 Tailsitter VTOL overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter
- PX4 fixed-wing position/TECS/NPFG tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 parameter reference:
  https://docs.px4.io/main/en/advanced_config/parameter_reference
