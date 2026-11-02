# R10: R9 Results And Next FW Damping Trial

2026-11-02. Private experimental SITL, not production or hardware validation.
Same aircraft/folder/plugin and diagnostic firmware; no new PX4 bug claim.

## Decision

Retain MC_ROLLRATE_P=0.060. R9 completed moderate MC flight, RTL and landing
without emergency braking or sustained roll oscillation. Do not claim large
back-transition recovery is proven: R9 contained no transition/FW flight.

For R10, change ONLY FW_RR_P from 0.07536 to 0.10. Keep its I/D, differential
scale0.55, FW_R_TC=1.5, all MC gains, speeds, TECS, pitch/back-transition settings,
ACF/CG/mass and motor mapping unchanged. This is a 33% FW P experiment, not an
identified optimum or a guaranteed complete solution. No production airframe
or official PX4 parameter file is changed.

## R9 Evidence

ZIP SHA256 ab397c11f96bffb4888c268520b7f3691f5046d780b9dfc654f7199735dee009.
Runner 2026-11-02.ref6.9; diagnostic PX4
9cfe130a8d8e0c25f88e044141c523247ca53d48, official base
f90b40d61e56405513dfcd8e194287ac60d3a6ae. Same ACF hash
c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
Actual mass 2.000000477 kg, zero payload/wind, zero ULog dropouts.

Startup overlays applied MC_ROLLRATE_P=0.060, MPC_YAW_MODE=5 and
MIS_TAKEOFF_ALT=50 before arming. ULog initial_parameters are recorded BEFORE
the overlay; changed_parameters plus the parameter gate verify the flown values.
Do not mistakenly report the initial 0.10 roll P or yaw mode0 as the flown tune.

| Event, PX4 boot time | Time s |
| --- | ---: |
| External/QGC takeoff request | 27.561 |
| First / second GoTo | 48.596 / 94.342 |
| RTL | 139.730 |
| RTL internal Land target | 241.578 |
| Touchdown / automatic disarm | 375.723 / 377.243 |

R9 had ZERO emergency-braking activations, versus 33 in R8. The Land phase
showed sustained descent and completed, unlike R8. No recurring bias/velocity/
compass warning was logged after the startup heading wait.

### Tracking Qualifications

The operator flew two longer GoTos, approximately 148 m and 104 m, then RTL.
The first was retargeted approximately 10.35 m before arrival. Its negative
along-track overshoot is therefore not a successful stopping-distance test.

| Metric | First GoTo | Second GoTo |
| --- | ---: | ---: |
| Actual roll range deg | -0.61 to +0.37 | -4.25 to +1.07 |
| Roll-rate error RMS rad/s | 0.0091 | 0.0208 |
| Roll-rate command RMS rad/s | 0.0039 | 0.0217 |
| Pitch-rate error RMS rad/s | 0.0339 | 0.0260 |
| Yaw-rate error RMS rad/s | 0.0125 | 0.0652 |
| Heading change from leg start deg | -1.25 to +1.06 | -7.96 to +9.35 |
| Cross-track RMS m | 0.14 | 6.46 |

First-leg roll demand was tiny. The second leg provides small-roll evidence,
not reproduction of R8's +/-30 degree post-back limit cycle. Routes, initial
conditions and modes differ: do not interpret the error numbers as an isolated
causal A/B proof that the new roll gain fixed all recovery behavior.

The second-leg yaw-rate response remains weak. Yaw integrator peaks around
0.01065 rather than its limit, with small torque commands/motor headroom.
That physical MC-yaw axis becomes FW roll. This is relevant evidence, but not
a reason to automatically double both mode gains without measuring FW.

### Vertical Estimate And Long RTL

Independent simulator-CSV versus estimated vertical-speed RMS is 0.00360 m/s.
Truth alignment altitude RMS is 0.00632 m at sim-minus-boot offset53.76629 s.
Accel-bias component RMS is approximately0.00268/0.00345/0.00951 m/s2.

QGC requested takeoff about10 m above home, overriding the scenario's console
takeoff height. RTL_RETURN_ALT=180 and RTL_DESCEND_ALT=150 are inherited from
the full-size test configuration. PX4 logged its return climb to180 m above
destination. Land then took about134 s at measured median1.401 m/s downward,
consistent with the configured1.4 m/s descent. This was not an estimator-induced
descent stall. Use current-position Land, not RTL, for the next short trial.

## Why Increase FW Roll P?

Independent Astra review rechecked the R8 raw ULog and CSV in cruise windows
130-145 and145-162 boot seconds, with FW roll = negative MC yaw.

| R8 oscillatory measurement | Window 1 | Window 2 |
| --- | ---: | ---: |
| Differential throttle phase behind torque command | 1.6 deg | 1.9 deg |
| Prop thrust-differential phase lag | 14.9 deg | 16.8 deg |
| Prop drag-torque differential phase lag | 15.2 deg | 17.0 deg |
| P-term amplitude after0.55 scaling | 0.01526 | 0.01641 |
| I-term amplitude, same scaling | 0.00093 | 0.00116 |
| D-term amplitude, same scaling | 0.00077 | 0.00090 |

At the observed oscillation frequency, the propulsion phase corresponds to
roughly0.10-0.12 s. This is closed-loop harmonic evidence, not a uniquely
identified transport delay. Differential propulsion shows neither an obvious
sign inversion nor near-180-degree lag. P dominates the oscillatory command
and opposes roll rate. A blind P reduction is not supported by these findings.

The FW roll integral was approximately-0.0223..+0.0341 versus its0.20 limit;
allocator-reported unallocated torque was zero in both windows. Those facts
do not establish physical authority, but do not support saturation as the cause.

Sideslip also oscillated, about1.6-1.8 degree harmonic amplitude. Prop differential
signals alone fit angular acceleration poorly: aerodynamic coupling is still
material, and physical cant/force correspondence remains unverified. Prop Q/T
changes from approximately0.0224 m in hover to0.053 m in FW. Equal normalized
PID coefficients never guaranteed equal physical closed-loop dynamics.

The chosen next step is FW_RR_P=0.10: effective physical-axis P becomes
0.10*0.55=0.055, versus0.041448. Not the larger proposed0.08 MC-yaw /
0.145455 FW-roll pair, because that would change front-transition excitation
and MC recovery as well as FW. MC yaw can be investigated separately later.

Do not change FW_R_TC first to mask insufficient inner-loop tracking; the
PX4 tuning sequence is rate control before outer-loop softness. This is a
data-supported experiment, not proof that a larger P must cure the oscillation.

The review is by an independent agent, not a human PX4 maintainer.

## Back-Transition Is Still A Separate Failure

R8 timer-ended at MC pitch-52.8 degrees, rather than reaching its approximately
-14.9 degree angle criterion. MC control was active during back-transition.
At175 s, requested pitch rate was+0.375 rad/s versus actual-0.159, despite
pitch torque0.0864 and an axial-thrust pitch-moment proxy about+0.70 N.m.
The proxy omits aerodynamic, cant and gyroscopic moments. The issue was not
simply absent differential command.

VT_B_TRANS_DUR controls BOTH the timeout and the pitch-program rate. Increasing
duration also slows the schedule. Initial throttle blending lasts0.5 s in the
pinned tailsitter; VT_B_TRANS_RAMP is not the operative knob for that blend.
Do not silently lengthen the timer, impose a collective floor, bypass health
checks or modify PX4 core as a substitute for understanding this response.

R10 retains these settings and measures recovery with the revised R9 MC roll
gain after the FW damping trial. Timer-only exit is still failure even if it
eventually lands. If the pitch criterion is not met, inspect pitch request,
actual rate, differential thrust/moments, speed, collective and coupling before
choosing the next change.

## Test And Verification

Follow QUICKSTART.md, copied to /home/alireza/PX4_REF6_CURRENT_TEST.md:
same-folder update, launch flight mode, CONSOLE takeoff to180 m, settled hover,
one front transition,15 s confirmed FW, one normal back-transition, settled MC,
then current-position Land and shutdown. No new GoTo, speed change or RTL.

Keep a large clear simulation area. Preserve failed intervals; do not wait
minutes in uncontrolled oscillation. Shutdown still snapshots truth and uploads
automatically. No manual ZIP copying, new token or aircraft installation.

Acceptance is comparative: lower FW bank/rate oscillation without worse
sideslip, saturation or altitude tracking; actual upright back recovery, settled
MC, and sustained Land descent. It is not a production/demo acceptance flight.

Host checks cover parameter values, unchanged ACF/directions, source-frame
contracts, update preservation and automatic evidence collection. They do not
replace a run on the operator's X-Plane system. Raw compressed captures remain
on the private server. Large analysis uses RAM and streaming, not full extraction.

## Reproduction And References

- tools/analyze_qtailsitter_tracking.py ZIP NEW_OUTPUT_DIR: MC legs, actual
  excitation, rate errors, yaw drift, braking counts, truth alignment and plots.
- tools/analyze_qtailsitter_handoff.py R8_ZIP NEW_OUTPUT_DIR: frame-correct FW,
  back-transition seconds, MC/Land/braking plots and local rate-model screens.
- [PX4 FW rate/attitude tuning](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing).
- [PX4 tuning troubleshooting](https://docs.px4.io/main/en/config/autotune_fw):
  distinguish inner-rate and outer-attitude behavior, not visual oscillation alone.
- [Pinned tailsitter source](https://github.com/PX4/PX4-Autopilot/blob/f90b40d61e56405513dfcd8e194287ac60d3a6ae/src/modules/vtol_att_control/tailsitter.cpp).
