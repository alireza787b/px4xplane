# Report v31 - Alia Takeoff Gate And Ehang Roll Recovery

Scope: review `/home/alireza/evtol5.zip`, keep the now-good Alia fixed-wing
behavior intact, recover Ehang from the v3.4.18 roll oscillation, and prepare
the next Alia/Ehang validation package.

## Evidence

`evtol5` contained two valid PX4 logs:

- `16_17_05.ulg`: Alia, `SYS_AUTOSTART=5020`, about `405 s`.
- `16_27_37.ulg`: Ehang 184, `SYS_AUTOSTART=5010`, about `155 s`.

TruthCapture also worked for both runs:

- Alia: `41,157` frames, mean callback rate about `69 Hz`, zero dropped rows,
  no sim-time reset.
- Ehang: `13,749` frames, mean callback rate about `91 Hz`, zero dropped rows,
  no sim-time reset.

## Alia Finding

Alia fixed-wing behavior is now the baseline to protect. The run had no PX4
health warnings, FW loiter/RTL looked aligned with the intended large-eVTOL
envelope, and the X-Plane log confirmed no default lift-prop braking:

- `No autoPropBrakes... Alia250`
- `Motor brakes configured for motors: 00000000`

TruthCapture showed the pusher was active in FW while the four lift motors were
near idle with low residual rotation. This means v3.4.19 does not change Alia
FW, TECS, NPFG, transition, airspeed, or prop-brake defaults.

The remaining Alia issue was MC takeoff acceptance. During Auto Takeoff, the
local altitude setpoint reached about `91.6 m` and then paused from roughly
`60 s` to `100 s` while horizontal error stayed above the default
`MPC_XY_ERR_MAX=2`. PX4 documents this parameter as the point where trajectory
setpoint integration stops to wait for the vehicle. For a heavy X-Plane VTOL,
that default is too strict during vertical takeoff.

Fix: set Alia `MPC_XY_ERR_MAX=10`, the PX4 maximum. This only changes the
multicopter takeoff/trajectory gate; it does not touch fixed-wing behavior.

## Ehang Finding

The Ehang failure is a tuning regression from v3.4.18, not an actuator mapping
or missing roll command. The log repeatedly reported `Attitude failure (roll)`.
Actual roll reached about `70 deg` while commanded roll was capped near
`25 deg`, and control allocation showed large unallocated roll/yaw torque with
motors saturating between `0` and `1`.

The immediate cause was the v3.4.18 jump from `MC_ROLL_P=0.3` and
`MC_PITCH_P=0.3` to `3.0`, combined with reduced horizontal velocity damping.
Older Ehang logs with the `0.3` attitude baseline had no roll-attitude failures.

Fix: roll Ehang back to the last non-oscillatory multicopter baseline:

- `MC_ROLL_P=0.3`, `MC_PITCH_P=0.3`, `MC_YAW_P=0.6`
- `MPC_XY_P=0.05`
- `MPC_XY_VEL_P_ACC=1.2`, `MPC_XY_VEL_D_ACC=1.5`
- `MPC_XY_CRUISE=5.0`, `MPC_XY_VEL_MAX=6.5`, `MPC_TILTMAX_AIR=30.0`

This is intentionally a recovery slice. Do not add another Ehang gain increase
until a clean retest proves the recovered baseline is stable.

## Plugin UX

The waiting connection HUD now shows the active airframe, for example
`Airframe: Alia 250 | Waiting for PX4`. The menu/data window use friendly names
such as `Ehang 184`, but the config section remains `ehang184` and the PX4 make
target remains `xplane_ehang184` for compatibility.

## Next Test

Use v3.4.19, run `make distclean` once after replacing/pulling the PX4 airframe
files, and verify:

- Alia PX4 log shows `SYS_AUTOSTART=5020` and `MPC_XY_ERR_MAX=10`.
- Ehang PX4 log shows `SYS_AUTOSTART=5010`, `MC_ROLL_P=0.3`,
  `MC_PITCH_P=0.3`, and `MPC_XY_VEL_D_ACC=1.5`.
- X-Plane log shows `px4xplane: Version: v3.4.19`.
- While waiting for SITL, the HUD shows the selected airframe.
- Alia takeoff reaches the assigned altitude without the long setpoint pause.
- Ehang has no roll attitude failure and no sustained large roll oscillation.
