# R13: Current PX4 Full-Flight Test

SITL only. This supersedes R12. The same aircraft, plugin and PID gains are used.
The runner now builds official PX4 main snapshot `649929706f` plus logging only
(`58794523a1`). The already-merged braking fix is included, not a new private fix.

## 1. Update The Same Folder

Stop PX4 and the old TruthCapture recording. Paste in WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect `2026-11-03.ref6.13`. The first update may finish by printing an OLD
R12 next-test line; ignore it and follow this guide. No new folder, aircraft,
plugin reinstall or upload token is needed.

## 2. Fresh Simulator Flight

Load a fresh QuadTailsitterRef6 flight with a fully charged X-Plane battery,
zero payload and no wind. Do not resume the prior depleted/airborne state.
Start a NEW TruthCapture recording after prepare-truth, then QGC.
Select Plugins > PX4 X-Plane > Airframes > QuadTailsitterRef6, then Connect to SITL.

In WSL:

```bash
bash run.sh flight --yes-debug
```

Use this plain flight command, not the old recovery-jerk or braking-isolation
cases. The runner prints and records both firmware pins. It imports new source
into your existing PX4 checkout, aligns submodules and builds. First build can
take time and download missing dependencies; if the plugin wait expires, click
Connect to SITL again. No complete PX4 clone is required.

## 3. PX4 Console Checks And Takeoff

After PARAMETER GATE PASSED and Ready for takeoff:

```text
ver all
param show MPC_JERK_AUTO
param show MPC_ACC_HOR
param show MPC_XY_VEL_MAX
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect firmware hash beginning `58794523a1`, values `4`, `2`, `4`, `180`.
The 2 kg ACF and rate PID values are unchanged. Do not override EKF checks.
Use console takeoff, not QGC's remembered takeoff slider. Wait for actual 180 m
above home, adequate terrain clearance and 15 seconds settled hover.

## 4. Front Transition And FW

```text
commander transition
listener vtol_vehicle_status -n 1
```

Issue once. Confirm vehicle_vtol_state: 4, then observe 20 seconds established
FW without changing the target or speed. Hold normally flies an orbit.
Current upstream includes turn-coordination/TECS/underspeed fixes, but smooth
FW flight is not yet demonstrated for this model.

Then request back-transition ONCE:

```text
commander transition
```

## 5. MC Recovery And Land

Confirm actual upright, stopped, settled MC, not just its label:

```text
listener vtol_vehicle_status -n 1
listener vehicle_local_position -n 1
```

If horizontal speed is approximately below 1 m/s and vertical speed near zero,
observe 15 seconds settled, then:

```text
commander land
```

After touchdown and automatic disarm:

```text
shutdown
```

Leave TruthCapture running until collection finishes. Wait for UPLOAD VERIFIED.
The runner restores your original checkout, compresses truth and uploads the
evidence automatically. No manual file copying is needed after success.
Retry a pending upload with `bash run.sh retry-uploads`.

## End A Failed Test Promptly

If recovery oscillates without damping for 15-20 seconds, use shutdown and send
that failure; do not leave it airborne for minutes. Also end for a growing
attitude departure, persistent motor clipping, about 30 m height loss or actual
horizontal speed above about 12 m/s in MC. Complete the run within six minutes.
These are experiment stop criteria, not certified aircraft limits.

No autotune, slider edits, GoTo, RTL or speed changes in this run. R13 changes
the official firmware AND normal trajectory settings (jerk 4, acceleration 2),
so improvement is not an isolated A/B proof for one PR. FW/MC PID, ACF, speeds,
trim and allocation remain fixed.

To undo recorder-only extra logging later, stop PX4 and the recording, then
run `bash run.sh restore-truth`. It does not alter simulator physics.
