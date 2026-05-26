# Report v58 - qtail22 FW Energy and Back-Transition Recovery

Date: 2026-05-26
Package target: `v3.4.46`

## Executive Summary

`qtail22.zip` is useful because it reached the first complete mission-style
fixed-wing evidence point. The multicopter baseline and front transition are no
longer the primary problem. The new problem is fixed-wing energy/guidance and
back-transition entry speed.

v3.4.46 keeps the qtail21/qtail22 MC lateral tuning, the 5 kg aircraft sizing,
the canted rotor geometry, and the stationary-ground sensor contract. It changes
only MC climb rate, fixed-wing speed/guidance, back-transition timing, and the
visual rendering of the large fixed contact gear.

## Evidence Reviewed

From `/home/alireza/qtail22.zip`:

- TruthCapture recorded `46,913` rows over about `545.7 s` sim time with `0`
  dropped rows, `0` sim-time resets, and about `86 Hz` mean callback rate.
- X-Plane loaded px4xplane `v3.4.45`, `Config Name: QuadTailsitter`,
  `Airspeed source=body_axis pitotAxisBody=-Z`, and the packaged
  `QuadTailsitter.acf`.
- PX4 loaded the intended qtail values: `SYS_AUTOSTART=5021`,
  `FW_USE_AIRSPD=1`, `SYS_HAS_NUM_ASPD=0`, `VT_ARSP_TRANS=24`,
  `FW_AIRSPD_TRIM=28`, `NPFG_PERIOD=34`, `NAV_LOITER_RAD=900`, and
  `VT_B_TRANS_DUR=8`.
- MC climb was slow by configuration, not by thrust deficit. The climb phase
  used about `0.8 m/s` actual climb with `MPC_Z_V_AUTO_UP=1.0`.
- Front transition succeeded. PX4 entered FW at about `195.5 s`.
- FW speed was too high for the current mission/guidance tune. TAS was commonly
  `35-40 m/s` while TECS was targeting `28 m/s`, so QGC throttle near zero was
  expected rather than evidence of missing motor output.
- Back-transition started while still too fast. During the back-transition
  window, TAS was about `37-55 m/s`, sink reached roughly `14-32 m/s`, and
  motors saturated near `1.0` before quad-chute/failsafe.

## Aircraft Sizing Check

The current ACF remains plausible for the next test:

- Empty mass is about `4.99 kg`; loaded truth mass was about `5.44 kg`.
- Wing area estimate is about `0.509 m^2`, giving loaded wing loading near
  `105 N/m^2`.
- Stall estimate is about `11-12 m/s` at `CLmax=1.2`.
- Required lift coefficient estimates are approximately `0.43` at `20 m/s`,
  `0.30` at `24 m/s`, `0.22` at `28 m/s`, and `0.12` at `38 m/s`.
- In the steady FW subset, X-Plane alpha was around `-86.8 deg`; for this
  tailsitter convention that corresponds to an effective wing AoA near `3 deg`.

Conclusion: qtail22 does not justify changing wing area, CG, prop size, motor
KV, rotor cant, or mass in this slice. The aircraft was not underpowered; it was
overspeeding and entering back-transition with too much kinetic energy.

## v3.4.46 Changes

- MC climb:
  - `MPC_Z_V_AUTO_UP` from `1.0` to `1.5`
  - `MPC_Z_VEL_MAX_UP` from `1.2` to `1.8`
- FW speed and transition energy:
  - `FW_AIRSPD_MIN` from `21` to `18`
  - `FW_AIRSPD_TRIM` from `28` to `24`
  - `FW_AIRSPD_MAX` from `40` to `34`
  - `VT_ARSP_BLEND` from `18` to `15`
  - `VT_ARSP_TRANS` from `24` to `21`
  - `VT_F_TRANS_THR` from `0.55` to `0.43`
- FW guidance and roll authority:
  - `NPFG_PERIOD` from `34` to `26`
  - `NPFG_DAMPING` from `1.00` to `0.90`
  - `NPFG_ROLL_TC` from `1.5` to `1.2`
  - `FW_R_LIM` from `28` to `32`
  - `FW_RR_P` from `0.20` to `0.28`
  - `FW_RR_I` from `0.10` to `0.08`
  - `FW_RR_FF` from `0.10` to `0.18`
  - `NAV_LOITER_RAD` and `RTL_LOITER_RAD` from `900` to `400`
- Back-transition:
  - `VT_B_TRANS_DUR` from `8` to `14`
  - `VT_B_DEC_MSS` from `1.2` to `0.8`
  - `VT_B_DEC_I` from `0.10` to `0.05`
  - `VT_B_TRANS_RAMP` from `6` to `10`
- ACF visual gear:
  - Hide rendered geometry for the four large fixed contact legs.
  - Keep the fixed gear physics/contact model active and non-retractable.

## Next Test

Use v3.4.46 for a manual FW/back-transition gate, not a full VTOL Land mission:

1. Pull the PX4 `px4xplane-sitl` branch and force a clean parameter load with
   `make distclean`.
2. Install the v3.4.46 px4xplane package and packaged `QuadTailsitter` aircraft.
3. Confirm X-Plane log shows `v3.4.46`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
4. Start TruthCapture before connecting PX4.
5. Take off, verify MC Go-To remains below about `6 m/s`, then climb to at
   least `120 m` AGL.
6. Command one forward transition and hold a straight FW leg or very large
   loiter only.
7. Manually back-transition while high; then land in MC.

Acceptance:

- no stationary/pause sensor warnings
- MC climb near `1.3-1.5 m/s`
- front transition reaches positive body-axis airspeed near `21 m/s`
- FW TAS mostly in the `22-32 m/s` band after stabilization
- no sustained roll bang-bang at the limits
- manual back-transition does not exceed about `5 m/s` sink and does not
  quad-chute

## References

- PX4 Tailsitter VTOL overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter
- PX4 fixed-wing position/NPFG and TECS tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
