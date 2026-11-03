# R15: Ref6 Candidate Validation

SITL ONLY. Same aircraft/plugin/firmware. FW_RR_P0.22 and modest faster MC
requests are candidates, not a production release. Full findings: REPORT.md.

## 1. Update The Same Folder

Stop PX4 and any old truth recording. WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect **2026-11-03.ref6.15**. No new folder, clone, plugin/ACF install or token.
If the old updater prints older next-test text, follow this guide.

## 2. Load First, Record Second

1. Load/reset a FRESH QuadTailsitterRef6 flight, fully charged simulator battery,
   zero wind/payload. Finish all aircraft/airport resets now.
2. Start a NEW TruthCapture recording. Leave it recording until collection ends.
   Do NOT reload/reset X-Plane or restart the capture during this run.
3. Open QGC. Select Ref6 in Plugins > PX4 X-Plane > Airframes, then Connect to SITL.

R14's ULog was usable but its truth window was rejected for a frame/time reset.
The runner must not fabricate or silently join incompatible truth segments.

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
param show MPC_XY_CRUISE
param show MPC_Z_V_AUTO_UP
param show MPC_Z_V_AUTO_DN
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect hash58794523a1..., values0.22,5,2.5,1.6,180. No EKF overrides.
Use CONSOLE takeoff: QGC's explicit altitude overrides the default.
Observe actual180m above home with terrain clearance, then15s settled hover.
Hover targets zero speed; 5m/s is the translation request, not hover motion.

## 4. Short MC Travel And Vertical Check

In QGC Fly View, click a clear point about100m away, choose Go To Location
and confirm. Keep the same altitude and allow it to arrive/stop. Then command
a second clear point about100m back, again allowing a complete stop. Don't
click a new target every few seconds: we need acceleration and stopping data.

Use QGC's Change Altitude action (or Pause's altitude control, depending on
version) for a20m climb and return to the original180m, waiting for each target.
Check the requested units/reference before confirming. QGC guided altitude
settings must permit these simulator heights. If that control isn't available,
skip ONLY this up/down pair and tell the reviewer; initial climb and final Land
still exercise vertical tracking, but the altitude-change item remains untested.

No gain, speed or parameter slider edits. After the final target, wait10s settled.

## 5. Front Transition And Clean FW Window

PX4 console:

```text
commander transition
listener vtol_vehicle_status -n 1
```

Request once. Confirm state4. Observe75s of established FW with NO retarget,
speed/altitude changes or RTL. A short matched25s interval alone hid the later
mode in R14; this longer window tests whether oscillation really damps.
A complete1300m-radius orbit is not expected in75s.

If bank oscillation grows, bank exceeds about30deg or roll rate repeatedly
exceeds35deg/s, request back-transition ONCE early instead of continuing FW.
Otherwise after75s:

```text
commander transition
```

## 6. Recover And Land

Allow up to25s of DECAYING attitude recovery, watching actual orientation,
height and motion. Hold may command travel back toward its previous target;
it need not be stationary to request Land. Once upright and attitude settled:

```text
commander land
```

Land should stop horizontal translation and descend near1.6m/s at altitude,
then slow toward0.9m/s near ground. The0.36m/s crawl applies only when its
distance-sensor condition is satisfied. Do not expect constant fast descent
through touchdown.

After touchdown and automatic disarm:

```text
shutdown
```

End the SIMULATOR test with shutdown for growing departure, non-decaying
recovery after25s, persistent motor clipping, about30m height loss, or MC
horizontal speed above12m/s. Keep airborne time under8min on a fresh battery.
These are experimental stop criteria, not certified hardware procedures.

## 7. Upload And Feedback

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
