# R10 Copy-Paste Commands

SITL only. Read QUICKSTART.md for the timing and observations.

WSL, PX4 stopped:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh flight --yes-debug
```

Expect 2026-11-02.ref6.10. Fresh Ref6 flight, no wind/payload; TruthCapture and
QGC running; select Ref6 and Connect to SITL in the X-Plane plugin.

PX4 console after parameter gate:

```text
param show FW_RR_P
param show MC_ROLLRATE_P
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect 0.10 / 0.060 / 180. Do not use QGC's remembered takeoff altitude.
At actual 180 m height and settled hover, once:

```text
commander transition
listener vtol_vehicle_status -n 1
```

When state 4 confirms FW, observe 15 seconds. Then once:

```text
commander transition
```

After actual upright, stopped, settled MC, observe 15 seconds, then:

```text
commander land
```

After touchdown/auto-disarm, or to end a failed simulation:

```text
shutdown
```

Wait for UPLOAD VERIFIED. No RTL, GoTo or gain changes in this short run.
Land is commander land, not commander mode land. Diagnostics if needed:

```text
mavlink status
listener vehicle_status -n 1
listener vehicle_local_position -n 1
listener vehicle_rates_setpoint -n 1
listener trajectory_setpoint -n 1
```
