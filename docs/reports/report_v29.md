# Report v29 - Alia Airspeed and Ehang Orbit Evidence

Date: 2026-05-19

Scope: review `/home/alireza/evtol3.zip`, identify why Alia showed zero
airspeed and never completed FW transition, explain the Ehang orbit/roll
observation, and patch the next validation package.

## Evidence Reviewed

The archive contained:

- `10_23_22.ulg`: short Alia startup/ready run, `SYS_AUTOSTART=5020`
- `10_25_02.ulg`: Alia flight attempt, `SYS_AUTOSTART=5020`
- `10_33_10.ulg`: Ehang flight, `SYS_AUTOSTART=5010`
- TruthCapture `20260519-102412Z`: Alia, `29,078` frames, zero dropped rows
- TruthCapture `20260519-103147Z`: Ehang, `54,694` frames, zero dropped rows
- X-Plane `Log.txt`: px4xplane `v3.4.16`

## Alia Finding

Alia did not lose raw airspeed from the bridge. In `10_25_02.ulg`:

- `differential_pressure` continued through the flight and reached about
  `2064 Pa`.
- raw `airspeed.indicated_airspeed_m_s` reached about `58.2 m/s`.
- `airspeed_validated` stopped publishing at `14.35 s`, just after arming and
  before the vehicle had useful forward speed.

That explains the visual symptom: QGC and Standard VTOL use validated airspeed,
so they saw stale near-zero airspeed even though raw pitot data later became
valid. The aircraft entered front transition but never completed FW mode and
quadchuted.

## Alia Fix

The Alia SITL virtual pitot now uses:

```sh
param set-default ASPD_DO_CHECKS 1
```

This keeps the missing-data check but disables validator checks that are not
useful while a Standard VTOL is intentionally hovering or climbing at near-zero
airspeed before front transition.

The PX4 fork also gets a defensive `airspeed_selector` patch. If the selected
airspeed source becomes disabled, ground-minus-wind, or synthetic, the selector
no longer indexes the physical-sensor validator array with an invalid index
while preparing `airspeed_validated`.

## Ehang Orbit Finding

The Ehang run did not show a missing roll channel. The QGC Orbit command in the
ULog requested about `146.47 m` radius and a center about `518 m` away from the
vehicle. The aircraft spent most of the observed Orbit mode flying to the
closest point on that circle:

- start radius from center: about `518 m`
- end radius before RTL: about `137 m`
- commanded radius: about `146 m`

At `5 m/s` and `146 m` radius, the lateral acceleration is only
`v^2 / r = 0.17 m/s^2`, which corresponds to roughly `1 deg` tilt. Seeing little
visible roll is expected for that large, slow passenger-style orbit. The log did
show pitch setpoints up to the multicopter tilt limit during the long capture,
but that is the vehicle flying mostly straight toward the circle, not proof that
roll mapping is broken.

## Connection Workflow Bug

The X-Plane log showed the plugin connecting, then immediately calling
`toggleEnable()` again and disconnecting after an aircraft/airframe change. The
cause is stale `lastFlightTime`: after a new aircraft loads, X-Plane sim time
can be lower than the previous run. The first connected frame was interpreted as
a sim-time reset, so the bridge disconnected itself.

v3.4.17 resets bridge timing state, including `lastFlightTime`, before opening a
new SITL server socket. This should remove the first-connect disconnect after
switching to Ehang.

## Next Test Notes

- For Alia, confirm the ULog contains `ASPD_DO_CHECKS=1`.
- Confirm `airspeed_validated` continues after arming and rises with raw
  `airspeed`.
- For Ehang orbit, command a center closer to the vehicle or wait long enough
  after capture before judging circular tracking.
- For a visibly banked Ehang orbit, use a smaller radius or higher speed only if
  that matches the test goal. Passenger-style validation should prefer the large,
  low-tilt orbit.
