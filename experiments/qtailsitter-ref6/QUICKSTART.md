# R9: Dampen MC Roll Before The Next Transition

SITL only. Same aircraft, same plugin, same pinned diagnostic firmware.
One gain change: MC_ROLLRATE_P 0.031815 -> 0.060; I/D and FW gains unchanged.
This is an experiment, not a verified full-flight tune. No hardware use.

## 1. Update In WSL

Stop PX4. X-Plane may stay open: aircraft/config bytes do not change.

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
```

Expected `2026-11-02.ref6.9`. No download/extraction, plugin install or token
configuration again. Start a fresh X-Plane flight if the previous one stopped
airborne; do not resume the previous uncontrolled aircraft state.

## 2. Prepare And Launch

Load the same QuadTailsitterRef6: 2 kg, zero payload, no wind, clear flat terrain.
Start TruthCapture and QGC. In X-Plane select Plugins > PX4 X-Plane > Airframes >
QuadTailsitterRef6, then Connect to SITL.

```bash
bash run.sh hover --yes-debug
```

This automatically commands 50 m console takeoff height and sets/checks
`MPC_YAW_MODE=5` for fixed heading. Value 4 is NOT fixed heading in this firmware.
The override is recorded in identity.json and applied only to this scenario.
The reusable parameter profile retains the base airframe's yaw mode 0.

## 3. Take Off

Wait for REF6 PARAMETER GATE PASSED and Ready for takeoff. In the PX4 console:

```text
param show MC_ROLLRATE_P
param show MPC_YAW_MODE
commander check
commander takeoff
```

Expect 0.060 and 5. Use console takeoff, not QGC's remembered takeoff slider.
Wait for 50 m and about 15 seconds of steady hover. Note the heading arrow in
QGC: it should keep pointing the same direction throughout the sideways test.

## 4. Two Lateral GoTos In QGC

1. In Fly view, select a point 10-15 m to the aircraft's LEFT or RIGHT relative
   to its heading arrow, not ahead of the arrow. Confirm Go To at the same height.
2. Let it arrive and hold for 15 seconds. Do not keep moving the target.
3. Go back to the starting point, still at the same height. Hold 15 seconds.

This deliberately excites roll without turning the nose toward each waypoint.
The previous successful GoTos mainly tested pitch. Do not transition or use RTL
in this short run. No further in-flight gain edits are needed.

If QGC is disconnected, do not substitute a stationary hover for lateral testing:
run `mavlink status` and preserve/report the connection issue instead.

## 5. Land And Upload

After the return settles, land at the current position:

```text
commander land
```

Let touchdown and automatic disarm complete, then:

```text
shutdown
```

Leave TruthCapture running. The runner snapshots and uploads automatically.
Wait for UPLOAD VERIFIED, then report that the test finished. Retry an upload:

```bash
bash run.sh retry-uploads
```

For a failed simulation, retain the oscillation interval and use shutdown;
do not spend minutes repeating uncontrolled recovery. About 10-15 seconds of
sustained oscillation after a GoTo is enough evidence. No force-arm or EKF-limit
changes. A process exit while airborne is not a successful landing.

## What We Compare

Actual/desired roll rates and attitudes; lateral overshoot/settling; torque,
motor headroom, yaw hold and coupling; emergency-braking count; raw versus
smoothed descent commands. Stable stationary hover alone is not a pass.
FW/back-transition follow after this result: their R8 gains remain available
but unvalidated, not silently replaced by the MC correction.
