# R14 Copy-Paste Commands

SITL only. Same R13 firmware/ACF; only FW_RR_P changes 0.14 -> 0.18.
Full scenario and stop criteria: QUICKSTART.md.

Stop PX4 and old TruthCapture recording. WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect 2026-11-03.ref6.14. Fresh charged Ref6, no wind/payload. Start NEW
TruthCapture recording, QGC and the Ref6 plugin connection. WSL:

```bash
bash run.sh flight --yes-debug
```

After PARAMETER GATE PASSED and Ready, PX4 console (NOT WSL):

```text
ver all
param show FW_RR_P
param show MPC_JERK_AUTO
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect 58794523a1..., 0.18, 4, 180. Use console takeoff, not QGC's remembered
altitude. At actual 180 m, terrain clearance and 15 s settled:

```text
commander transition
listener vtol_vehicle_status -n 1
```

Confirm state 4. Observe 25 s; continue to 45 s total FW only if bounded.
Then back-transition ONCE:

```text
commander transition
```

Allow up to 25 s of DECAYING attitude recovery. Hold may command 3.5 m/s travel
toward its old target; zero XY speed is not required to request Land.
Once upright with settled attitude:

```text
commander land
```

After touchdown/autodisarm, or to end a failed SIMULATOR experiment:

```text
shutdown
```

Keep truth recording until collection ends; wait for UPLOAD VERIFIED.
Network failure: retry the SAME evidence, no repeat flight:

```bash
bash run.sh retry-uploads
```

No in-flight edits, autotune, GoTo, RTL or speed changes. Maximum seven minutes
airborne. Growing departure, about 30 m height loss, persistent motor clipping,
MC speed above 12 m/s or non-decaying recovery: end the test. Early FW stop
thresholds and back-transition handling are in QUICKSTART.md.
