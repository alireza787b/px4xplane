# R16: Ref6 Damping And Energy Test

SITL ONLY. Same aircraft/plugin/firmware. Four tuning changes, applied
automatically and checked before arming. Findings and tradeoffs: REPORT.md.

## 1. Update The Same Folder

Stop PX4 and any old truth recording. WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect **2026-11-03.ref6.16**. No new folder, clone, plugin/ACF install or token.
If the old updater prints older next-test text, follow this guide.

## 2. Load First, Record Second

1. Load/reset a FRESH QuadTailsitterRef6 flight, fully charged simulator battery,
   zero wind/payload. Finish all aircraft/airport resets now.
2. Start a NEW TruthCapture recording. Leave it recording until collection ends.
   Do NOT reload/reset X-Plane or restart the capture during this run.
3. Open QGC. Select Ref6 in Plugins > PX4 X-Plane > Airframes, then Connect to SITL.

R15 had a complete usable truth recording. Keep this load-then-record order;
the runner must not silently join incompatible truth segments.

WSL:

```bash
bash run.sh flight --yes-debug
```

Reconnect the plugin if a build outlasts its connection wait. Dirty-checkout
errors now show changed paths; don't reset/clean blindly.

## 3. Check And Take Off

After PARAMETER GATE PASSED and Ready, type in the PX4 console, NOT WSL:

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

Expect hash58794523a1..., values0.26,0.16,0.10,0.18,5,2.5,1.6,180.
No EKF overrides or manual tuning edits.
Use CONSOLE takeoff: QGC's explicit altitude overrides the default.
Observe actual180m above home with terrain clearance, then15s settled hover.
Hover targets zero speed; 5m/s is the translation request, not hover motion.

## 4. Short MC Travel And Stop Check

In QGC Fly View, click a clear point about60m away, choose Go To Location
and confirm. Keep the same altitude and allow it to arrive/stop. Then command
a second clear point about60m back, again allowing a complete stop. Don't
click a new target every few seconds: we need acceleration and stopping data.

Initial takeoff and final Land exercise the retained2.5/1.6m/s MC requests.
After the final travel target, wait10s settled. No gain/speed slider edits.

## 5. Front Transition And Clean FW Window

PX4 console:

```text
commander transition
listener vtol_vehicle_status -n 1
```

Request once. Confirm state4. Observe60s of established FW with NO retarget,
speed/altitude changes or RTL. This is long enough to check more than the first
25s after entry. A complete1300m-radius orbit is not expected in60s.

If bank oscillation grows, bank exceeds about30deg or roll rate repeatedly
exceeds35deg/s, request back-transition ONCE early instead of continuing FW.
If stable, continue to step6. If not, skip that step and back-transition early.

## 6. Small FW Altitude Step

In QGC, use Change Altitude (or Pause's altitude control, depending on version)
to request200m above home:20m above the intended180m level. Confirm the
altitude units/reference before accepting. QGC's guided altitude limits must
permit this simulator height. Wait30s without new commands. We need this
energy-loop response, not only a quiet cruise segment. If the action is not
available, report this item as skipped; do not improvise console commands.

Then request back-transition ONCE in the PX4 console:

```text
commander transition
```

## 7. Recover And Land

Allow up to25s of DECAYING attitude recovery, watching actual orientation,
height and motion. Hold may command travel back toward its previous target;
it need not be stationary to request Land. Once upright and attitude settled:

```text
commander land
```

This test lands at the current position; it does not require RTL. Keep terrain
clearance throughout. Land should stop horizontal translation and descend near1.6m/s at altitude,
then slow toward0.9m/s near ground. The0.36m/s crawl applies only when its
distance-sensor condition is satisfied. Do not expect constant fast descent
through touchdown.

After touchdown and automatic disarm:

```text
shutdown
```

End the SIMULATOR test with shutdown for growing departure, non-decaying
recovery after25s, persistent motor clipping, about30m UNCOMMANDED height loss, or MC
horizontal speed above12m/s. Keep airborne time under8min on a fresh battery.
These are experimental stop criteria, not certified hardware procedures.

## 8. Upload And Feedback

Keep truth recording until collection finishes. Wait for UPLOAD VERIFIED.
Network failure does NOT require another flight; WSL:

```bash
bash run.sh retry-uploads
```

Tell the reviewer the final status. The ZIP goes to the ANALYSIS SERVER, not
necessarily the VPS home folder. No manual copy is needed after success.
Keep original recordings until their integrity is checked.

Review must confirm FW damping, new MC speed/stop behavior, back-transition
climb/recovery and successful Land. Good visuals alone do not validate stall,
maximum sustainable speed, all weather/FPS or the complete production envelope.
