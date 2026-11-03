# R12 Copy-Paste Commands

Detailed sequence and stop conditions: QUICKSTART.md. Experimental SITL only.

PX4 stopped; stop the old TruthCapture recording:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect `2026-11-03.ref6.12`. Fresh Ref6 flight, full X-Plane battery, no payload
or wind. Start a new TruthCapture recording and QGC; connect the Ref6 plugin.

```bash
bash run.sh flight --scenario recovery-jerk --yes-debug
```

After the parameter gate, in the PX4 console:

```text
param show MPC_JERK_AUTO
param show MPC_XY_VEL_MAX
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect `4`, `4`, `180`. Wait for actual 180 m above home and 15 seconds settled:

```text
commander transition
listener vtol_vehicle_status -n 1
```

When state 4 is confirmed, count 15 seconds. Then once:

```text
commander transition
```

Wait for actual upright/stopped MC, then 15 seconds settled:

```text
commander land
```

After automatic disarm, or after 15-20 seconds of sustained failed recovery:

```text
shutdown
```

Wait for UPLOAD VERIFIED. No RTL, GoTo, autotune or gain edits in this test.
Use `bash run.sh retry-uploads` if upload fails. Keep the original Windows truth
recording until the analysis is complete. No file copying is needed after upload.
