# R13 Copy-Paste Commands

SITL only. Full sequence/stop criteria: QUICKSTART.md. This replaces R12.

Stop PX4 and old TruthCapture recording; in WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect 2026-11-03.ref6.13. Fresh fully charged Ref6, no wind/payload. Start NEW
TruthCapture recording, QGC and plugin connection. Then in WSL:

```bash
bash run.sh flight --yes-debug
```

Wait for build/connection and PARAMETER GATE PASSED. PX4 console:

```text
ver all
param show MPC_JERK_AUTO
param show MPC_ACC_HOR
param show MPC_XY_VEL_MAX
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect hash 58794523a1..., values 4, 2, 4, 180. At actual 180 m plus 15 s settled:

```text
commander transition
listener vtol_vehicle_status -n 1
```

FW state 4, then 20 s established FW. Back-transition once:

```text
commander transition
```

After actual upright/stopped MC plus 15 s settled:

```text
commander land
```

After touchdown/autodisarm, or 15-20 s sustained failed recovery:

```text
shutdown
```

Wait for UPLOAD VERIFIED; leave TruthCapture running until collection ends.
No in-flight edits, autotune, GoTo, RTL or speed changes. Retry failed uploads:
`bash run.sh retry-uploads`.
