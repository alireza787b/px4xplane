# R8: First Short FW And Back-Transition Test

SITL ONLY. MC baseline passed the last two GoTos and RTL; FW gains are initial
test seeds, not flight-validated. Same aircraft, same folder, no plugin install.

## 1. Update In WSL

Stop PX4. X-Plane may stay open for this parameter-only update.

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
```

Expected `2026-11-02.ref6.8`. No token setup again. Do not change gains manually.

## 2. Prepare X-Plane

- Same QuadTailsitterRef6 aircraft, 2 kg total, zero payload, no wind.
- Choose flat terrain with 150-200 m actual ground clearance during FW testing
  and a kilometre-scale clear recovery area. A height above home is not terrain
  clearance over rising ground. Do not test among buildings or hills.
- Start TruthCapture recording and open QGC.
- Plugins > PX4 X-Plane > Airframes > QuadTailsitterRef6; Connect to SITL.

## 3. Launch

```bash
bash run.sh flight --yes-debug
```

Wait for REF6 PARAMETER GATE PASSED and Ready for takeoff. In the PX4 console:

```text
commander check
commander takeoff
```

Use THIS console takeoff, not QGC's remembered takeoff altitude. It commands
180 m above the local home reference. At 2 m/s this climb takes about 90 seconds.
Wait for controlled hover at height, then observe 20 seconds. Verify QGC is
connected, estimate is healthy and the aircraft is not drifting/oscillating.

## 4. Front Transition, Five Seconds FW

```text
commander transition
```

Issue ONCE. Watch the transition; confirm actual FW mode and finite airspeed
near 25-26 m/s. Do NOT count five seconds from the command: count from confirmed
FW. First exposure is FIVE seconds only; no new GoTo, orbit exercise, RTL,
speed change or prolonged FW flight. A shallow turn from Hold may begin; do not
try to complete its large orbit.

If control is satisfactory after five seconds, request normal MC recovery:

```text
commander transition
```

Issue ONCE. Back-rotation is now scheduled over 15 seconds, not five. It can
travel hundreds of metres while slowing. Do not command landing immediately
and do not assume that a mode label means it is upright or stopped.

## 5. Recover, Then Land

Wait until visibly upright, controlled in MC, horizontal speed below about
1 m/s and vertical speed near zero. Then observe another 20 seconds of stable
hover. You can check the state in the PX4 console:

```text
listener vtol_vehicle_status -n 1
listener vehicle_local_position -n 1
```

Require `vehicle_vtol_state: 3`; use vx/vy/vz and QGC for actual movement.
Then land AT THE CURRENT POSITION over clear ground, not RTL:

```text
commander land
```

Let controlled descent, touchdown and automatic disarm complete. Then:

```text
shutdown
```

Leave TruthCapture running. Wait for UPLOAD VERIFIED and tell me the run is
done. No manual ZIP copying. A failed upload can be retried:

```bash
bash run.sh retry-uploads
```

## Stop Conditions

If oscillation grows, attitude departs uncontrolled, motor output clips
persistently, airspeed becomes invalid/below 21 m/s in established FW, or height
drops roughly 20 m, do not continue the exercise or wait for a crash. End the
simulation and preserve the failed interval with shutdown. These thresholds are
test limits, not guaranteed recovery boundaries. Do not force-arm or relax EKF
limits. Do not repeatedly toggle modes. Report any quad-chute separately.

This run is for one short front/FW/back sequence. Longer FW, turns and faster
MC speed limits come after evidence review. Current MC speeds remain unchanged
so the recovery controller is the same one you just tested successfully.
