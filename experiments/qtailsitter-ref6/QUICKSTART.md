# R14: FW Damping And Landing Check

Experimental SITL ONLY. R13 landed but still has a FW roll mode, back-transition
climb/timeout and recovery transient. R14 changes only FW_RR_P 0.14 -> 0.18.
Same firmware, aircraft and MC settings. Read REPORT.md for remaining gaps;
this is not production acceptance.

## 1. Update In WSL

Stop PX4 and the old TruthCapture recording:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect **2026-11-03.ref6.14**. Follow THIS guide if the old updater prints an
older next-test line. No new folder, upload token, ACF or plugin installation.

## 2. Fresh X-Plane Flight

Load QuadTailsitterRef6 with a fresh fully charged battery, zero wind/payload.
Do not resume the old depleted/airborne state. Start a NEW TruthCapture recording
after prepare-truth, then QGC. Select Plugins > PX4 X-Plane > Airframes >
QuadTailsitterRef6, then Connect to SITL.

WSL:

```bash
bash run.sh flight --yes-debug
```

Existing PX4 checkout is reused and restored afterward. If the build outlasts
the plugin connection wait, click Connect to SITL again.

## 3. PX4 Console Checks And Takeoff

After PARAMETER GATE PASSED and Ready, type in the PX4 console, NOT WSL:

```text
ver all
param show FW_RR_P
param show MPC_JERK_AUTO
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect hash beginning 58794523a1 and values 0.18, 4, 180. Stop if identity or
checks differ; no EKF overrides. Use CONSOLE takeoff: QGC previously supplied
an explicit approximately 57 m target, overriding the 180 m default.
Wait for actual 180 m above home, terrain clearance and 15 s settled hover.

## 4. Front Transition And FW

```text
commander transition
listener vtol_vehicle_status -n 1
```

Issue once; confirm vehicle_vtol_state: 4. Observe the first 25 s, then continue
to 45 s total established FW only if motion remains bounded. Look for reduced
or damping oscillation, not simply more bank. A complete 1300 m orbit is NOT
expected in 45 s.

If bank oscillation clearly grows, actual bank exceeds about 30 deg, or roll
rate repeatedly exceeds about 35 deg/s, request back-transition ONCE early
instead of continuing FW. These are experimental stop thresholds, not aircraft
design limits or a substitute for observing the flight.

Otherwise, after 45 s:

```text
commander transition
```

## 5. MC Recovery And Land

Allow up to 25 s of a DECAYING attitude transient. Observe actual orientation,
not just the MC label. R13 entered MC by timeout at about -20 deg pitch and
9.3 m/s residual speed, then took about 19 s to settle attitude. This remains
under review, not a claimed fixed transition.

Hold may command a smooth 3.5 m/s return toward its old waypoint. That translation
is not necessarily unstable hover. Once upright with settled attitude, Land
should stop the horizontal travel at the landing position:

```text
commander land
```

Expect about 1.4 m/s descent at altitude and slower near-ground descent. After
touchdown and automatic disarm:

```text
shutdown
```

End the SIMULATOR experiment with shutdown instead if recovery grows or does
not decay within 25 s, motor clipping persists, height loss is about 30 m or
MC XY speed exceeds about 12 m/s. Keep airborne time below seven minutes.
These are diagnostic limits, NOT certified hardware procedures.

## 6. Upload

Keep TruthCapture recording until collection ends. Wait for UPLOAD VERIFIED.
The destination is the analysis server, not the VPS home folder. No manual
copy is needed. On network failure, preserve the ZIP and run folder; WSL:

```bash
bash run.sh retry-uploads
```

Tell the reviewing agent the retry completed and paste its final status. No
repeat flight solely for an upload failure. No autotune, gain edits, GoTo, RTL
or speed changes in this comparison. A later measured mission will address
full path capture, left/right turns and the performance envelope.
