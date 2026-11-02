# R9 Operator Commands

See QUICKSTART.md for the current MC-only lateral test. SITL only.

WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh hover --yes-debug
```

PX4 console, after parameter gate and Ready for takeoff:

```text
param show MC_ROLLRATE_P
param show MPC_YAW_MODE
commander check
commander takeoff
```

Expect gain 0.060, fixed-yaw mode 5 and 50 m takeoff height. In QGC make a
10-15 m SIDEWAYS GoTo, settle, then return, settle. Do not transition or use RTL.

```text
commander land
```

After touchdown and automatic disarm, or to end a failed simulation:

```text
shutdown
```

Wait for UPLOAD VERIFIED. TruthCapture need not be stopped manually. If needed:

```bash
bash run.sh retry-uploads
```

Diagnostics, if movement/descent is unexpected:

```text
mavlink status
listener vehicle_status -n 1
listener vehicle_local_position -n 1
listener vehicle_rates_setpoint -n 1
listener trajectory_setpoint -n 1
```

Land is `commander land`, not `commander mode land`. Hold, when specifically
requested in a later scenario, is `commander mode auto:loiter`, not
`commander hold`. Do not repeatedly toggle modes or relax health checks.
