# R12: Recovery Handoff Test

SITL only. Same Ref6 aircraft, plugin and diagnostic firmware. No new PID,
geometry, motor-direction, EKF, TECS or cruise-speed changes. This is a diagnostic
test, not a promised complete-flight fix. R11 already supplies the baseline.

## 1. Update And Add Read-Only Measurements

Stop PX4 and stop any current TruthCapture recording. In WSL:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect `2026-11-03.ref6.12`. No extraction, plugin reinstall, new aircraft or token.
`prepare-truth` appends a reversible block to TruthCapture's custom dataref list:
prop/aerodynamic/total moments and forces, runtime inertia and battery measurements.
It does not write physics or change the px4xplane config. Existing custom entries
are preserved. To undo later: `bash run.sh restore-truth` with PX4 stopped.

## 2. Fresh Aircraft And Capture

Start a fresh Ref6 flight with a fully charged X-Plane battery, zero payload and
no wind. Do not resume the previous stopped-airborne/depleted-battery state.
Keep a large, flat recovery area clear. Start a NEW TruthCapture recording after
step 1, then start QGC. Select Plugins > PX4 X-Plane > Airframes >
QuadTailsitterRef6, then Connect to SITL.

```bash
bash run.sh flight --scenario recovery-jerk --yes-debug
```

This changes ONLY `MPC_JERK_AUTO=4` from the R11 baseline `0.7`. The runner checks
the actual override and records it. All R11 PID values remain unchanged.
Missing moment/energy datarefs stop the diagnostic before changing the PX4 checkout.
Do not repeat baseline unless requested: `flight` without `--scenario` intentionally
retains the old baseline, not the new recovery experiment.

## 3. PX4 Console: Take Off

After PARAMETER GATE PASSED and Ready for takeoff:

```text
param show MPC_JERK_AUTO
param show MPC_XY_VEL_MAX
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect `4`, `4`, `180`. Use this takeoff command, NOT QGC's remembered takeoff
slider. Wait for actual height 180 m above home, adequate terrain clearance and
15 seconds of settled hover. No GoTo, RTL, autotune or in-flight parameter edits.

## 4. Front Transition, Then 15 Seconds FW

```text
commander transition
listener vtol_vehicle_status -n 1
```

Issue the transition once. Confirm `vehicle_vtol_state: 4`, then count 15 seconds
from established FW. Its inherited Hold orbit may turn gently; leave the target
alone. FW oscillation is NOT claimed fixed by changing an MC trajectory limit.
The extra prop/aerodynamic moments are intended to diagnose it.

After those 15 seconds, request normal back-transition once:

```text
commander transition
```

## 5. Recovery And Land

Wait for upright, settled MC with horizontal speed approximately below 1 m/s and
vertical speed near zero. The MC mode label alone is not sufficient.

```text
listener vtol_vehicle_status -n 1
listener vehicle_local_position -n 1
```

If it settles, observe another 15 seconds, then:

```text
commander land
```

After touchdown and automatic disarm:

```text
shutdown
```

Leave TruthCapture running until collection finishes. Wait for UPLOAD VERIFIED.
Truth compression, collection and upload are automatic. Retry a retained upload:

```bash
bash run.sh retry-uploads
```

## Stop A Failed Test Promptly

If recovery keeps oscillating for 15-20 seconds without damping, stop with
`shutdown` and send that failure. Do not leave it oscillating for minutes or tune
sliders. The prior run eventually exhausted available propulsion power near
800 seconds; that is a separate event, not evidence about early control quality.
Also stop for a growing attitude departure, persistent motor clipping, roughly
30 m height loss, or actual horizontal speed exceeding about 12 m/s in MC.
End the complete run within six minutes even if no clean landing is possible.
These are experiment stop criteria, not certified recovery limits.

## Second Case Only If Requested After Review

A separate fresh flight can isolate braking without altering PID:

```bash
bash run.sh flight --scenario braking-isolation --yes-debug
```

This starts again from R11 and changes ONLY `MPC_XY_VEL_MAX=12`; jerk remains `0.7`
and cruise remains `3.5`. It raises the braking trigger from `5.2` to `15.6 m/s`
and can suppress INITIAL braking as well as retriggers. It is not a production
fix, speed recommendation or permission to fly faster. Do not combine the cases
or interpret it as a pure on/off switch. Follow the same short scenario/stop
criteria, using a newly charged aircraft and recording.
