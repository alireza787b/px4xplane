# R10: Short FW Roll-Damping Test

SITL only. Same aircraft, plugin, firmware pin and R9 MC gains.
Only FW_RR_P changes: 0.07536 -> 0.10. FW/back recovery is not yet validated.

## 1. Update In WSL

Stop PX4. No aircraft/config bytes change, so X-Plane can stay open for updating.

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
```

Expect `2026-11-02.ref6.10`. No token, extraction or plugin reinstall.
Use the flight command below even if the OLD updater process prints hover.
Start a fresh X-Plane flight, not a previous stopped-airborne state.

## 2. Prepare And Launch

Load QuadTailsitterRef6, 2 kg, zero payload, no wind, flat terrain with a large
clear recovery area. Start TruthCapture recording and QGC. Select Plugins >
PX4 X-Plane > Airframes > QuadTailsitterRef6, then Connect to SITL.

```bash
bash run.sh flight --yes-debug
```

This uses the SAME diagnostic PX4 pin and 180 m console takeoff height. It does
not apply private dive/optional control fixes. Normal flight yaw mode is 0;
the R9 lateral-test yaw-fixed override only applies to the hover scenario.

## 3. Take Off From The PX4 Console

After REF6 PARAMETER GATE PASSED and Ready for takeoff:

```text
param show FW_RR_P
param show MC_ROLLRATE_P
param show MIS_TAKEOFF_ALT
commander check
commander takeoff
```

Expect 0.10, 0.060 and 180. Use THIS takeoff command, not QGC's remembered
takeoff slider. The last R9 QGC takeoff requested only about 10 m.
Wait until actual height is 180 m above home and hold is settled for 15 seconds.
Over flat terrain, verify actual ground clearance as well; home-relative height
does not guarantee clearance over hills.

## 4. Transition And Measure FW For 15 Seconds

```text
commander transition
```

Issue once. Confirm established FW:

```text
listener vtol_vehicle_status -n 1
```

Require vehicle_vtol_state 4. Start counting FIFTEEN seconds from confirmed FW,
not from the transition command. No GoTo, speed change, orbit completion or RTL.
Hold can start a shallow turn on its inherited 1300 m orbit: leave its target
alone during the measurement. Watch bank oscillation amplitude and altitude.

After 15 seconds of FW, request normal MC recovery once:

```text
commander transition
```

## 5. Recover, Then Land At Current Position

Back-rotation duration remains 15 seconds. Allow room for travel. A change to
MC mode does not prove it reached upright attitude or stopped.

```text
listener vtol_vehicle_status -n 1
listener vehicle_local_position -n 1
```

Require state 3 AND upright, settled attitude, horizontal speed roughly below
1 m/s and vertical speed near zero. Observe another 15 seconds of steady hover,
then:

```text
commander land
```

No RTL: its inherited 180 m return / 150 m descend heights add unnecessary
climb and a long descent to this short control test.

After touchdown and automatic disarm:

```text
shutdown
```

Leave TruthCapture running. Wait for UPLOAD VERIFIED. No manual ZIP copying.
A failed upload can be retried:

```bash
bash run.sh retry-uploads
```

## Failed Runs Are Useful

If oscillation grows markedly, FW bank departs beyond approximately 60 degrees,
height drops roughly 30 m, motors clip persistently, or the aircraft clearly
loses control, preserve the failure and end with shutdown rather than waiting
for a prolonged uncontrolled flight. These are simulation experiment limits,
not proven recovery boundaries. About 10-15 seconds of sustained failed MC
recovery is enough evidence; no need to leave it oscillating for minutes.

Do not relax EKF gates, force-arm, repeatedly toggle modes or change gains during
this run. A timer-only back exit or shutdown while airborne remains a failed
recovery, even if the FW segment improved. Compare FW bank/rate errors, sideslip,
actual torque/prop response, pitch recovery, braking and landing separately.
