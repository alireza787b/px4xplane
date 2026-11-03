# R15 Copy-Paste Commands

Same debug folder/aircraft/firmware. Full scenario: QUICKSTART.md.
Stop PX4 and old truth recording. WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect2026-11-03.ref6.15. FIRST load/reset a fresh charged Ref6 flight, no wind/
payload. THEN start NEW TruthCapture recording; no reload/reset afterward.
Open QGC and plugin Ref6 connection. WSL:

```bash
bash run.sh flight --yes-debug
```

After parameter gate/Ready, PX4 console:

```text
ver all
param show FW_RR_P
param show MPC_XY_CRUISE
param show MPC_Z_V_AUTO_UP
param show MPC_Z_V_AUTO_DN
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect58794523a1...,0.22,5,2.5,1.6,180. Actual180m, clearance,15s settled.
QGC: one100m MC GoTo and stop; another100m return and stop. One20m altitude
up/down pair if available; check requested altitude reference. Wait10s settled.
No parameter edits. Then PX4 console:

```text
commander transition
listener vtol_vehicle_status -n 1
```

FWstate4: observe75s, NO FW retarget/RTL/speed or altitude changes.
Then back-transition ONCE:

```text
commander transition
```

Up to25s DECAYING recovery; Hold translation can be intentional.
Once upright with settled attitude:

```text
commander land
```

Touchdown/autodisarm:

```text
shutdown
```

Keep recording until collection ends. WaitUPLOAD VERIFIED. Retry if needed:

```bash
bash run.sh retry-uploads
```

Abort criteria, early FW back-transition and skipped QGC altitude-test reporting
are in QUICKSTART.md. No new token/folder/install. Maximum8min airborne.
This is candidate validation, not a production/hardware release.
