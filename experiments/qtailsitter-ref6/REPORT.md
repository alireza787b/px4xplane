# R8: MC Results And First Short FW/Back-Transition Trial

2026-11-02.ref6.8. Experimental SITL only. No production, hardware or perfect-
flight claim. Official PX4, production plugin and the original aircraft are
unchanged. Same diagnostic pin and same experimental aircraft/folder.

## Latest Evidence

ZIP SHA256 e7a95ae888bc62d03eee78d629ba5cd1eb6057bfd190a384a13f89a650ed408e.
Runner r7, PX4 logging-only diagnostic 9cfe130a8d8e0c25f88e044141c523247ca53d48,
official base f90b40d61e. ACF SHA256
c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
Actual mass 2.000000477 kg; payload zero; wind zero; no ULog dropouts.
All VTOL states were MC: this log contains no transition/FW validation.

| Metric | Result |
| --- | --- |
| First GoTo length / overshoot | 63.51 m / 2.274 m |
| Second GoTo length / overshoot | 68.39 m / 3.023 m |
| Cross-track RMS, legs 1 / 2 | 0.298 / 1.025 m |
| First-leg pitch-rate error RMS | 0.0422 rad/s, previously 0.1483 |
| First-leg attitude error RMS | 4.858 degrees, previously 17.049 |
| Second-leg attitude error RMS | 8.788 degrees; still room for improvement |
| First-leg horizontal velocity error RMS | approximately 0.730 m/s, previously 2.478 |
| Independent estimator-vs-truth vertical velocity RMS | 0.00252 m/s |
| Truth-clock altitude-fit RMS | 0.00562 m, no vertical bias fitted |
| Health | No recurring bias/velocity/compass warning after startup heading wait |
| RTL / Land target / landed detection / disarm | 133.567 / 187.817 / 286.812 / 288.33 s boot |

The five QGC GoTo deliveries are retries of TWO horizontal targets, not five
separate manoeuvres. The analysis now groups consecutive same-target deliveries
within two seconds, allowing 0.1 m altitude retransmission variation. Raw events
remain in metrics. RTL stays in nav_state 5 while its internal target becomes
Land: the analysis uses target type, not only the nav_state 18 shortcut.

The first-leg comparisons use the same [first GoTo +1,+35] time window rule.
Routes, heights and hover seeds differ, so this is improvement consistent with
the pitch-P change, not an isolated causal identification or an optimum claim.

## Why It Looked Slow

The inherited MC limits are intentionally conservative: cruise 3.5 m/s, maximum
4 m/s, horizontal acceleration 0.7 m/s2, jerk 0.7 m/s3, automatic climb 2 m/s,
automatic descent 1.4 m/s, and yaw-rate/acceleration limits 25 deg/s and 8 deg/s2.
Heading mode also controls when heading changes are requested; slow direction
changes are not automatically weak motor authority.

The GoTo reached approximately 3.97 m/s: the configured limit was active. RTL
commanded 1623.994 m MSL, 129.916 m above home, then a long descent near 1.4 m/s.
The RTL-climb phase was predominantly vertical; Land began while still about
64 m horizontally from home, so the Land phase also includes horizontal return.
The 1.394 m/s median is NOT touchdown speed: descent slows near the ground and
truth vertical speed approaches zero before landed detection.

We can increase cruise/acceleration/climb/descent limits later, after FW recovery
and torque tracking are characterized. Changing them now would confound the next
mode-handoff test. Slow GoTo cruise does not directly set back-transition pitch
rotation or establish stopping distance. No RTL is used in the next short test,
avoiding another unnecessary return-altitude climb.

## R8 Parameter Changes

No MC gains, position limits, EKF settings, motor mapping/orientation, rotation
signs, CG, mass, geometry or propulsion bytes change. Their successful baseline
is retained. FW PID seeds use the same physical rotor axes and the pinned PX4
frame transforms, accounting for downstream differential-thrust scale:

    FW rates = (-MC yaw, MC pitch, MC roll)
    MC motor torque = (S_Y*FW yaw, S_P*FW pitch, -S_R*FW roll)

The two roll sign changes cancel. Keep all gains positive; do not reverse any
motor or axis. A compiled contract executes the actual pinned source's rate
transform, torque back-transform and differential routing, not just a hand-drawn
diagram. It verifies the unsaturated coefficient mapping for P/I/D separately.

| FW family | MC source / scale | P | I | D |
| --- | --- | ---: | ---: | ---: |
| FW_RR | MC yaw / 0.55 | 0.075360 | 0.014318 | 0.001507 |
| FW_PR | MC pitch / 0.90 | 0.133333 | 0.016667 | 0.001333 |
| FW_YR | MC roll / 0.32 | 0.099422 | 0.024856 | 0.001987 |

- FW_ARSP_SCALE_EN=0 removes the aerodynamic-surface inverse-speed-squared rate
  output multiplier. This aircraft has no movable control surfaces. Propeller
  authority still depends on collective/inflow, so constant gains are only a
  starting trial, not universal rotor performance compensation.
- FW_USE_AIRSPD=1 retains pitot airspeed for TECS, coordination and transition.
  It is NOT disabled to hide an airspeed problem.
- FW_THR_MIN=0.08 uses the same normalized collective baseline as MPC_THR_MIN,
  avoiding zero-collective commands in the first FW trial. It is NOT a verified
  physical idle thrust, per-motor minimum, or sufficient torque margin.
- VT_B_TRANS_DUR=15 reduces requested back-rotation from 18 to 6 deg/s. With the
  unchanged MC pitch attitude gain 0.58 and IDEAL instantaneous rate tracking,
  ramp-end lag is 90/(0.58*T)*(1-exp(-0.58*T)) degrees: about 15.5 at 10 s and
  10.4 at 15 s, against the -0.26 rad exit threshold (~14.9 degrees). This explains
  the trial choice; it is not a flight prediction. More time means more travel.
- Differential scales 0.55/0.90/0.32, zero rate FF, zero roll-to-yaw torque FF
  and enabled airspeed use are written explicitly, not silently inherited.

Cruise remains 26 m/s, transition CAS 25, front duration 10 s, bank limit 16
degrees, NPFG period 55, loiter radius 1300 m, trim throttle 0.25 and max throttle
1. TECS/NPFG performance is not inferred from MC data or retuned speculatively.

### Limits Of The Transfer

This equates normalized unsaturated PID coefficients, NOT complete dynamics.
FW compression, clipping, integral reset/limits and aerodynamic damping differ.
MC yaw has a 2 Hz torque-output filter not reproduced by transferred FW roll.
FW integral ceilings in motor axes differ from MC; they are NOT raised to force
equivalence. FW pitch outer-loop gain 1/0.6=1.67 also differs from MC 0.58.
The MC result is therefore a defensible seed, not verified FW stability.

The existing tailsitter back-transition still exits on angle OR duration, with
no speed gate. The inherited VT_B_TRANS_RAMP parameter does not control its
internal 0.5 s thrust blend. Timer-only exit is not accepted as successful
recovery merely because it eventually lands. Logging-only tail_diag identifies
the exit reason. No new PX4 bug or PR is asserted here.

## Next Trial

Read QUICKSTART.md. Set no wind and choose flat terrain with 150-200 m actual
AGL and a kilometre-scale clear recovery area. Use console takeoff to 180 m,
settled MC hold, one front transition, FIVE seconds established FW, then one
explicit normal MC transition. Do not complete an orbit or invoke RTL. Do not
issue Land until upright MC, residual horizontal speed below approximately
1 m/s, near-level vertical speed and 20 seconds of settled hold.

Stop escalation for growing oscillation, large uncontrolled attitude error,
sustained motor/allocator saturation, invalid airspeed, established FW CAS below
21 m/s, or approximately 20 m altitude loss. These are experiment limits, not
proven recovery guarantees. Save failures; no force-arm, sensor bypass or new
in-flight tuning. A quad-chute recovery is not a normal back-transition pass.

After shutdown, the existing runner snapshots truth and uploads automatically.
Required evidence already includes actual/desired attitude and rates, virtual
MC/FW torque, allocated torque/thrust, motor/RPM/shaft power, airspeed, gain
compression and tail_diag. Review pitch/roll reversal, actual handoff thrust,
TECS speed/height requests, and the angle-versus-timer exit. Only after this
short exposure passes should FW time extend to 15-20 seconds and gentle turns.

## Reproduction And References

`tools/analyze_qtailsitter_tracking.py LOG.zip NEW_OUTPUT_DIR` (numpy/pyulog)
streams truth, retains raw retry events, handles RTL Land targets, fits time
without extrapolation, and writes metrics.json/rates.svg/tracking.svg.

- [PX4 controller diagrams](https://docs.px4.io/v1.18/en/flight_stack/controller_diagrams): frames and aerodynamic airspeed scaling; documented disable option for non-surface-dependent controls.
- [PX4 MC rate/attitude tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html).
- [PX4 back-transition tuning](https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning.html).
- Actual behavior is checked against pinned source, not assumed from generic
  Standard-VTOL guidance: FixedwingRateControl.cpp, tailsitter.cpp, rate_control.cpp.
- Independent Astra review checked frame/gain-transfer limits and back timing;
  a separate evidence review checked MC metrics, command retries and RTL phases.
  These are agent reviews, not human PX4 maintainer approval.
