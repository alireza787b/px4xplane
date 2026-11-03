# R16 Copy-Paste Commands

Same debug folder/aircraft/firmware. Full scenario: QUICKSTART.md.
Stop PX4 and old truth recording. WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect2026-11-03.ref6.16. FIRST load/reset a fresh charged Ref6 flight, no wind/
payload. THEN start NEW TruthCapture recording; no reload/reset afterward.
Open QGC and plugin Ref6 connection. WSL:

```bash
bash run.sh flight --yes-debug
```

After parameter gate/Ready, PX4 console:

```text
ver all
param show FW_RR_P
param show MC_PITCHRATE_P
param show FW_T_I_GAIN_PIT
param show FW_THR_TRIM
param show MPC_XY_CRUISE
param show MPC_Z_V_AUTO_UP
param show MPC_Z_V_AUTO_DN
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect58794523a1...,0.26,0.16,0.10,0.18,5,2.5,1.6,180.
Actual180m, clearance,15s settled.
QGC: one60m MC GoTo and stop; another60m return and stop. Wait10s settled.
No parameter edits. Then PX4 console:

```text
commander transition
listener vtol_vehicle_status -n 1
```

FWstate4: observe60s without retarget/RTL/speed or altitude changes.
Then QGC Change Altitude to200m above home,20m above the intended180m level;
check units/reference, wait30s with no new command. If that action is unavailable,
report this part as skipped. Then back-transition ONCE:

```text
commander transition
```

Up to25s DECAYING recovery; Hold translation can be intentional. No RTL needed.
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
