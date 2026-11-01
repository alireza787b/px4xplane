# R7 Review: MC Hover Passed, GoTo Pitch Tracking Needs Work

Revision 2026-11-01.ref6.7. Experimental SITL only; no production-readiness,
optimal-gain, hardware-suitability or all-weather/platform guarantee.

## Evidence Identity

- ZIP SHA256: 3d88f2ccc5a1384aac9780713c0d9a801b4e1e1281e4070956fe20326ac601ba.
- Recorded runner r6, mode A; PX4 diagnostic commit
  9cfe130a8d8e0c25f88e044141c523247ca53d48, base f90b40d61e.
- ACF SHA256 c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
- Same original motor orientations, quadrant/channel ordering and prop signs.
- Actual mass 2.000000477 kg, fixed payload zero, measured wind zero.
- All recorded VTOL states were MC (3). No FW or transition occurred.

The user used QGC takeoff, not the 10 m console scenario: recorded target altitude
was 1513.079 m MSL, approximately 19 m over the local origin. This does not
invalidate the flight, but absolute climb duration is not a like-for-like 10 m
comparison. Use the console takeoff next time to keep the scenario repeatable.

## What Worked

| Measurement | Result |
| --- | --- |
| Takeoff request / takeoff detected | 15.606 / 17.590 s boot |
| Quiet hover attitude error RMS | 0.153 degrees, window 43.16-53.16 s |
| Quiet hover pitch rate error RMS | 0.00831 rad/s |
| GoTo command / Land command | 54.160 / 91.558 s |
| Touchdown / disarm | 111.035 / 112.553 s |
| Landing attitude error RMS | 0.525 degrees, window 93.56-109.04 s |
| Independent truth altitude clock-fit RMS | 0.00189 m, no vertical offset fitted |
| EKF vertical velocity versus aligned truth RMS | 0.00321 m/s |
| Accelerometer bias maxima, absolute, x/y/z | approximately 0.00186 / 0.00416 / 0.00989 m/s2 |
| Health messages | Startup heading-reference wait; no recurring in-flight bias/velocity/compass fault |

These confirm that this run's mass correction, basic hover and Land worked.
They do not prove the EKF pipeline is perfect for every flight condition. No
EKF thresholds, sensor scaling or PX4 core changes are justified by this record.

## What Still Needs Improvement

The GoTo was about 57.34 m. Cross-track error was 0.286 m RMS, maximum absolute
0.547 m: the main issue was NOT a gross lateral route error. Along-track
overshoot was about 8.00 m. It returned to within approximately 0.62 m of the
target before Land. In the 55.16-89.16 s comparison window:

| Measurement | Result |
| --- | --- |
| Pitch rate command-tracking error RMS | 0.1483 rad/s |
| Roll / yaw rate error RMS | 0.01615 / 0.02770 rad/s |
| Attitude command error RMS / peak | 17.05 / 35.70 degrees, predominantly pitch |
| Pitch rate cross-correlation lag | 1.38 s, correlation 0.780 |
| Motor commands | Large remaining headroom; approximately 0.228-0.290 in 55-85 s |
| Allocator torque achievement | 100% in the reviewed 55-85 s window |

The axial propeller pitch-moment proxy sum(-hub_Z * thrust), after independent
truth-clock alignment, correlates with requested torque at 0.996, with best
fit lag about 0.125 s. It omits cant-side forces, aerodynamic and gyroscopic
moments. Therefore the much longer rate-tracking lag should not be described as
a measured 1.38 s motor transport delay. Neither correlation identifies rigid
body inertia, crossover frequency or phase margin.

Independent review reproduced the weak pitch tracking and found integral/P
cancellation during some reversals, but not integral-limit windup: peak integral
approximately 0.0188 versus the 0.30 limit. There is no evidence-based reason to
raise integral limits, invert axes, reorder motors, change CG, or add ballast.

## Changes For The Next Test

Only TWO parameter values change from r6:

| Parameter | R6 | R7 | Reason |
| --- | ---: | ---: | --- |
| MC_PITCHRATE_P | 0.06 | 0.12 | Controlled P-only intermediate step for weak pitch-rate tracking |
| MPC_THR_HOVER | 0.25 | 0.26 | Actual 2 kg HTE medians: hover 0.2657, GoTo 0.2613, Land 0.2603 |

Pitch I=0.015, D=0.0012 and K=1 stay unchanged. So do roll/yaw gains, attitude
and position loops, acceleration/jerk/speed limits, aircraft bytes, CG, propeller,
power cap, EKF, FW, TECS and transitions. A 0.18 P trial was considered; independent
review recommended the smaller 0.12 step because headroom and correlations do
not establish gain margin. This is not a proven optimal gain.

The parameter descriptions now correctly state that VT_F_TRANS_THR and
VT_ARSP_BLEND are not effective tailsitter throttle/blending knobs in this pinned
code. Their values are unchanged; no effect is falsely credited to them.

The workflow remains the same: update the existing folder, reuse upload setup,
same aircraft, automatic truth/archive/upload after shutdown. No new core fix,
plugin binary, calibration bypass, aircraft variant or token step is required.

## Next Test And Acceptance

`bash run.sh A`: console takeoff to 10 m, 20 seconds of stable hover, one QGC
GoTo 40-60 m away at the same altitude, let it settle for 15 seconds, then Land.
A return GoTo is optional only if the first is controlled. Do not change speed
or gains in flight. Land/disarm, shutdown, wait for UPLOAD VERIFIED.

Compare against the same phase definitions: pitch-rate error and reversal
settling should improve, along-track overshoot should decrease, and hover,
cross-track, altitude and landing must not degrade. Reject new sustained
oscillation, growing attitude error or motor/allocator saturation. A flight
which merely arrives eventually is not enough. Stop uncontrolled simulation;
do not force arm or relax estimator limits.

## FW And Back-Transition Plan

This log cannot tune an unflown mode. Read-only review of the pinned source found:

| Area | Verified code behavior / next requirement |
| --- | --- |
| Differential rate control | FW roll uses MC-body yaw authority; pitch stays pitch. R/P/Y differential factors 0.55/0.90/0.32 apply after FW control. Do not blindly copy MC gains or invert axes. |
| Airspeed scaling | Inherited FW_ARSP_SCALE_EN=1 scales PID by roughly (26/max(CAS,17))^2: 1.53 at 21, 1.08 at 25, 0.585 at 34 m/s. Rotor-only authority versus collective/speed remains unmeasured. |
| Front transition | VT_F_TRANS_DUR=10 gives a 9 deg/s attitude schedule. Thrust comes from MC control; VT_F_TRANS_THR is not used by this tailsitter path. Completion uses attitude and finite CAS, not a Standard-VTOL blend schedule. |
| Back-transition | VT_B_TRANS_DUR=5 schedules 18 deg/s and can force MC entry after 5 s without a speed exit gate. VT_B_TRANS_RAMP is not its 0.5 s internal thrust blend. A timed mode change does not prove physical recovery. |
| Energy control | FW trim 0.25 and climb/sink capability are provisional. Throttle slew 0.2/s and minimum collective require examination with real FW evidence. Do not invent TECS tuning from hover. |
| Landing/recovery | MC Land success does not validate normal back-transition or quad-chute recovery. Retain NAV_FORCE_VT behavior; disabled optional quad-chute angle/altitude-loss guards are not new protection. |

After MC tracking passes, identify FW roll/pitch response near 26 m/s with small
separate demands and full truth/rate/torque/collective/allocator logging. A
recoverable FW-initialized SITL scenario can isolate this from front-transition
behavior; it is not implemented as a beginner command yet. Then test one
straight front transition at ample clearance, followed by a separately reviewed
altitude-held back-transition with room to stop. Establish upright MC recovery
and settled hold BEFORE requesting Land. A combined mission/RTL/landing demo
is premature while the inner-loop and FW response are still unknown.

No additional PX4 PR is supported by this flight. All changes remain on the
experimental px4xplane validation branch, not the production or official PR branch.

## Reproduction And References

`tools/analyze_qtailsitter_tracking.py LOG.zip NEW_OUTPUT_DIR` requires numpy and
pyulog. It verifies phase identities, derives command-based windows, computes
metrics, fits truth time without extrapolation, and writes rates.svg/tracking.svg.
It reads the ZIP without extracting full artifacts. Closed-loop data is explicitly
not labeled open-loop identification. Raw source/evidence remain preserved.

- [PX4 MC tuning guide](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html): inner rate loop before outer control.
- [PX4 back-transition guide](https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning.html).
- Pinned source: src/modules/vtol_att_control/tailsitter.cpp; src/modules/fw_rate_control/FixedwingRateControl.cpp; src/lib/rate_control/rate_control.cpp.
- Automatic simulator inertia and the 500 W derated 10x8 surrogate still need
  characterization. Manufacturer 5.889 kgf is for 10x5x3 at 2636 W electrical input,
  not our propeller/power point; no full-burst motor fidelity claim is made.
