# Report v28 - Alia Transition Brake and Ehang Orbit Tuning

Date: 2026-05-19

Scope: review `/home/alireza/evtol2.zip`, answer the Ehang orbit-capture
question, identify why Alia sank after front transition, update the generic
prop-brake mechanism, and prepare the v3.4.16 Alia/Ehang retest package.

## Evidence Reviewed

`evtol2.zip` contained one Alia run and one Ehang run:

- Alia ULog: `08_37_18.ulg`, `SYS_AUTOSTART=5020`
- Ehang ULog: `08_44_06.ulg`, `SYS_AUTOSTART=5010`
- Alia TruthCapture run: `20260519-083704Z`
- Ehang TruthCapture run: `20260519-084317Z`
- X-Plane logs showing px4xplane `v3.4.15`

TruthCapture quality remained good:

- Alia: `22,867` frames, about `324.9 s`, zero dropped rows, no sim-time reset.
- Ehang: `58,202` frames, about `648.3 s`, zero dropped rows, no sim-time reset.

## Ehang Orbit Finding

The Ehang orbit was not a bridge actuator-mapping bug. The vehicle was commanded
into Orbit while it was outside the commanded circle. PX4 Orbit mode first flies
to the closest point on the commanded circle before settling into the orbit, so
a short zigzag/S-turn during capture is expected when the command is started
from outside the radius.

The remaining issue was post-capture smoothness. The ULog showed the orbit
radius hunting around the commanded radius and velocity setpoints spending too
much time near the horizontal cap. v3.4.16 therefore reduces Ehang horizontal
aggression:

- `MPC_XY_CRUISE`: `6.0 -> 5.0`
- `MPC_XY_VEL_MAX`: `8.0 -> 6.5`
- `MPC_VEL_MANUAL`: `6.0 -> 5.0`
- `MPC_ACC_HOR`: `2.0 -> 1.5`
- `MPC_ACC_HOR_MAX`: `2.0 -> 1.8`

This keeps the airframe passenger-style and should make Orbit capture less
sharp without masking real navigation problems.

## Alia Front-Transition Failure

Alia flew normally through takeoff and front transition, then sank after the
fixed-wing state became active. In the first fixed-wing window, TECS was already
asking for maximum throttle and nose-up pitch/elevator, but the aircraft did
not recover enough altitude and eventually quadchuted.

The important comparison is against the previous accepted Alia run:

- v3.4.13 had no active lift-prop brake. The lift props windmilled slowly in FW,
  but the aircraft recovered from the initial sink.
- v3.4.15 hard-locked the lift props after the command/dwell/airspeed gates.
  Around that brake window, TruthCapture showed the lift prop speed forced near
  zero and prop pitch forced high. The aircraft did not recover and quadchuted.

The first part of the sink starts before the brake is fully active, so the root
cause is not "brake only." The brake did, however, correlate with losing the
remaining recovery margin. The safest next slice is to avoid invasive
flightmodel writes by default and increase the PX4 transition margin.

## Prop-Brake Decision

PX4 Standard VTOL already turns off upward motors during fixed-wing flight. The
bridge should therefore stay command-driven and generic: it should brake any
configured X-Plane motor only when PX4 has commanded that motor low long enough,
and it should release all configured brakes as soon as any of those motors gets
a recovery command.

v3.4.16 adds a configurable brake mechanism:

- `autoPropBrakeMode = feather`
  - writes normal X-Plane prop actuator requests only.
- `autoPropBrakeMode = hard_lock`
  - keeps the previous invasive low-level flightmodel prop writes available for
    controlled comparison, but not as the default.
- `autoPropBrakeMode = prop_separate`
  - uses X-Plane prop-separation failure datarefs as a controlled A/B option.

The Alia default for the next test is:

```ini
autoPropBrakes = 0, 1, 2, 3
autoPropBrakeApplyThreshold = 0.01
autoPropBrakeReleaseThreshold = 0.011
autoPropBrakeDwellSec = 8.0
autoPropBrakeMinAirspeedMps = 52.0
autoPropBrakeMode = feather
autoPropBrakeUseFailure = false
```

This is not hard-coded to Alia. Any future lift-plus-cruise eVTOL can opt in by
listing its X-Plane motor indices and choosing the brake mode.

## Alia PX4 Parameter Changes

v3.4.16 keeps the bank/radius recovery from v3.4.15 and adds transition margin:

- `VT_ARSP_TRANS`: `46.0 -> 50.0`
- `FW_PSP_OFF`: `3.0 -> 4.0`

The reason is direct: PX4 documentation warns that a Standard VTOL can lose
height at fixed-wing switch if the airspeed is not comfortably above stall
margin. The latest log showed the aircraft switching to FW too close to the
observed sink/recovery boundary.

## Transition Orbit Center

The bridge does not choose the transition orbit center. PX4 Navigator does.
Depending on the command path, it comes from the active mission/VTOL-takeoff
setpoint, reposition/loiter setpoint, or Orbit command center. Parameters such
as `NAV_LOITER_RAD` and `RTL_LOITER_RAD` change the radius, not the center.

For the next Alia test, use a long straight outbound segment before expecting
FW loiter accuracy. If using mission VTOL takeoff, place the post-transition
loiter/fixed-wing waypoint farther ahead so the aircraft is not forced into a
turn while still building wing-borne margin.

## References

- PX4 QuadPlane transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration
- PX4 Orbit mode:
  https://docs.px4.io/main/en/flight_modes_mc/orbit
- PX4 actuator notes for VTOL motor shutdown:
  https://docs.px4.io/main/en/config/actuators
- X-Plane prop mode/pitch behavior:
  https://developer.x-plane.com/article/understanding-override_prop_pitch-and-override_prop_mode/
- BETA ALIA public aircraft page:
  https://beta.team/aircraft

## Next Test Acceptance

- Ehang Orbit may show a short capture S-turn if started outside the radius, but
  should settle without repeated velocity-cap hunting.
- Alia should not quadchute after FW switch.
- Alia should recover the initial FW height loss with lower sink rate than
  `evtol2`.
- X-Plane log should show `mode=feather`, not `hard_lock`.
- Keep TruthCapture running so prop speed/pitch/torque can prove whether
  `feather` is enough or whether `prop_separate` deserves a controlled A/B run.
