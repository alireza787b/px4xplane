# Report v33 - v3.4.20 Regression and evtol6 Follow-Up

Date: 2026-05-20

Scope: review `/home/alireza/evtol6.zip`, explain why v3.4.20 broke the SITL
connection workflow, keep the accepted Alia fixed-wing behavior protected, and
prepare the next Alia/Ehang validation package.

## Evidence Reviewed

`evtol6` contains working v3.4.19 flight evidence:

- `04_54_11.ulg`: Alia, `SYS_AUTOSTART=5020`, full takeoff, transition, RTL,
  back-transition, landing, and auto-disarm.
- `05_01_47.ulg`: Alia with user-changed `VT_B_DEC_MSS=1.5`.
- `05_08_48.ulg`: Ehang 184, `SYS_AUTOSTART=5010`, takeoff, manual/stabilized
  checks, Orbit, RTL, landing, and manual disarm.
- TruthCapture had zero dropped rows and no sim-time resets.

The archive does not contain a v3.4.20 X-Plane log. The v3.4.20 failure is
therefore judged from the code delta plus the user's terminal/QGC observation.

## v3.4.20 Connection Regression

v3.4.20 made the accepted PX4 TCP stream non-blocking and then dropped a
complete MAVLink packet whenever `send()` returned would-block. That is unsafe
for PX4's simulator TCP link because PX4 expects the simulator stream to remain
ordered and complete after the handshake. The symptoms match this failure mode:
PX4 connected, but QGC parameter/telemetry loading stalled while commands could
still sometimes pass.

Fix in v3.4.21:

- keep the non-blocking listening socket and cancellable waiting UI,
- restore the accepted simulator socket to the v3.4.19 blocking behavior,
- stop dropping MAVLink packets on transient send back-pressure,
- stop turning normal send/receive edge cases into an immediate disconnect.

## Alia Findings

The Alia fixed-wing tune remains the baseline to protect. In `04_54_11.ulg`,
`airspeed_validated` stayed live through the flight and came from sensor source
1. The aircraft still loses altitude immediately after the fixed-wing switch,
with peak downward velocity about `6.8 m/s`, but it recovers and completes the
workflow. This is not enough evidence to retune TECS/transition again without a
targeted A/B test.

The airspeed preflight warning is a startup health-check race, not an in-flight
airspeed loss. PX4's airspeed health check reports `Airspeed selector module
down` when `SYS_HAS_NUM_ASPD > 0` and no fresh `airspeed_validated` sample has
arrived in the last two seconds. The Alia ULogs show the virtual pitot does
publish correctly once the simulator stream is warm. v3.4.21 keeps
`FW_USE_AIRSPD=1` and `ASPD_DO_CHECKS=1`, but sets `SYS_HAS_NUM_ASPD=0` so the
virtual SITL pitot is not treated as a prearm hardware requirement.

The long post-touchdown disarm was land-detector state bouncing. In both Alia
and Ehang logs, `landed=true` later dropped false because the local vertical
velocity estimate briefly reached about `0.50-0.54 m/s`, just above the
effective threshold from `LNDMC_Z_VEL_MAX=0.20`. PX4 land-detector guidance
requires no vertical movement and says `MPC_LAND_CRWL` must be greater than
`LNDMC_Z_VEL_MAX`; v3.4.21 uses `MPC_LAND_CRWL=0.30` and
`LNDMC_Z_VEL_MAX=0.24`.

The user's back-transition test did not actually contain `VT_B_TRANS_DUR=10`;
the ULog still showed `35.0`. It did contain `VT_B_DEC_MSS=1.5`, and the
back-transition lasted longer and travelled farther, matching PX4 guidance that
lower expected deceleration starts the transition farther from the destination.
v3.4.21 adopts only `VT_B_DEC_MSS=1.5` and leaves `VT_B_TRANS_DUR=35.0`.

## Ehang Findings

Ehang still had the same takeoff acceptance issue Alia previously had:
`MPC_XY_ERR_MAX=2` held Auto Takeoff for about `109 s` while XY error was around
`5-6 m`. v3.4.21 sets Ehang `MPC_XY_ERR_MAX=10`.

The Orbit problem is not a missing roll channel. The Orbit command requested
about `143 m` radius and started about `128 m` from the center; the vehicle
crossed the radius briefly, then was sent to RTL after only about `38 s` in
Orbit. Pitch setpoint repeatedly hit `+-30 deg`, while actual pitch lagged near
`+-17 deg`; roll setpoints were smaller. This points to under-responsive
attitude response plus too much horizontal velocity D damping, not a mapping
failure. v3.4.21 makes a conservative change:

- `MC_ROLL_P=0.45`
- `MC_PITCH_P=0.45`
- `MPC_XY_VEL_D_ACC=0.8`

This is intentionally far below the failed v3.4.18 jump to `3.0`.

## v3.4.21 Test Sanity

For Alia, confirm:

- `MPC_XY_ERR_MAX=10`
- `SYS_HAS_NUM_ASPD=0`, `ASPD_DO_CHECKS=1`
- `MPC_LAND_CRWL=0.30`, `LNDMC_Z_VEL_MAX=0.24`
- `VT_B_TRANS_DUR=35.0`, `VT_B_DEC_MSS=1.5`
- `autoPropBrakes` remains empty

For Ehang, confirm:

- `MPC_XY_ERR_MAX=10`
- `MC_ROLL_P=0.45`, `MC_PITCH_P=0.45`
- `MPC_XY_VEL_D_ACC=0.8`
- `MPC_LAND_CRWL=0.30`, `LNDMC_Z_VEL_MAX=0.24`

## References

- PX4 parameter reference for `MPC_XY_ERR_MAX`, `MPC_LAND_CRWL`, and
  `MPC_XY_VEL_D_ACC`: https://docs.px4.io/v1.15/en/advanced_config/parameter_reference
- PX4 land-detector configuration:
  https://docs.px4.io/v1.15/en/advanced_config/land_detector.html
- PX4 back-transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
