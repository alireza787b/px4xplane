# R14 Review, R15 Candidate And Production Plan

Experimental Ref6 SITL only. R14 successfully returned and landed. It is not
yet evidence of a fully damped fixed-wing envelope or a production-ready tune.
No production plugin, original aircraft, PX4 bridge code or public PR is changed.

## Evidence And Limitations

- Archive SHA256: ac38730ff2338e69d324172da7ef8bb6c21500db1f297d1f7b4e84b612b89cff.
- 41,591,249 bytes; ZIP CRCs and all26 evidence-file hashes pass.
- ULog: 2026-11-03/10_27_19.ulg, firmware58794523a1, R14 profile.
- All82 generated profile values match reconstructed startup parameters.
  No in-flight parameter edits or ULog dropouts.
- ACF hash stays c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
- No independent truth frames are attached. The recorder existed at preflight,
  but collection rejected non-monotonic frame/time or a reset during the window.
  This is not ZIP corruption and does not invalidate the ULog. It prevents
  independent force/moment/battery or sensor-pipeline verification for this run.
- Both requested independent AI reviewers hit the usage limit before returning
  results. The current analysis and decisions were completed locally; no
  completed expert review or human maintainer approval is claimed.

Do not change snapshot logic to silently accept/reset-stitch bad timestamps.
Load/reset the fresh X-Plane flight BEFORE starting the new truth recording,
and do not reload/reset it until shutdown and collection finish.

## What R14 Actually Tested

All times below are PX4 boot seconds.

| Event | Time s | Meaning |
| --- | ---: | --- |
| Airborne | 18.826 | External QGC takeoff request |
| Hold | 75.187 | Actual takeoff target about98m above home |
| Front / FW | 82.159 / 94.875 | Normal front transition |
| Altitude commands | 114.971 / 176.496 | Two QGC reposition/altitude requests |
| RTL | 234.476 | Navigator commands return at180m above destination |
| Back / MC | 293.420 / 308.163 | Normal RTL back-transition |
| RTL Land target | 345.460 | Land happens inside RTL, not nav_state18 |
| Landed / auto-disarm | 468.324 / 469.851 | Successful landing and disarm |

QGC again supplied an explicit altitude, overriding MIS_TAKEOFF_ALT180.
The flight also continued FW for about199s, changed altitude and used RTL.
These are useful additional observations, but not the exact R14 short
comparison scenario. Do not claim an isolated causal A/B.

## FW Findings

The first matched24.7s roll-rate-error RMS is0.122rad/s (~6.97deg/s), versus
R13's0.175rad/s (~10.01deg/s). Dominant roll-rate amplitude is about0.064rad/s
(~3.65deg/s) versus roughly0.19rad/s in R13. This supports an early improvement,
not universal damping; entry altitude and command history differ.

The longer record shows a persistent approximately0.46Hz roll mode:
- FW135-175s: roll-rate-error RMS0.216rad/s, harmonic amplitude0.271rad/s.
- FW190-230s: errorRMS0.194rad/s, amplitude0.258rad/s (~14.8deg/s).
- RTL240-293s: errorRMS0.244rad/s, amplitude0.321rad/s.
- The late135-230s bank oscillation is about5deg amplitude; the corresponding
  bank-command amplitude is only about0.38-0.50deg. It is not simply tracking
  an equally large oscillating guidance command.

Motor commands remain approximately0.019-0.501 over the full FW interval.
Motor allocator torque residual stays below6.1e-8 and no actuator saturation
codes are reported. Roll integral peaks around0.043 against limit0.2.
There is room for a bounded damping change, not evidence for larger thrust
limits, integral limits, new geometry or estimator overrides.

R13 native moment evidence showed opposing-rate prop/P work near this mode.
That supports a further small P-only test, but R14 has no native moment capture
to independently confirm the same physical balance. Closed-loop correlations
do not identify actuator delay, aerodynamic derivatives or stability margins.

TECS follows the commanded altitude changes and RTL climb rather than a
continuous uncontrolled FW climb. Speeds oscillate with the flight and must be
reviewed together with lateral motion. No pitch/energy-limit retune is inferred
from the absence of a steady calibrated trim maneuver. Keep trim/speed/TECS
seeds fixed for the next trial.

## MC, Recovery And Landing

MC climb35-60s tracks about2.001m/s; rate-error RMSxyz~0.0072/0.0073/0.0083rad/s.
Stable high-altitude Land averages1.391m/s. The existing initial and near-ground
speed limits explain slow commanded movement; there is no evidence here that
the controller rejected those commands.

Back-transition slows about30.2 to9.2m/s and gains about37m of height.
It exits by ANGLE this time, at14.734s and pitch-14.62deg, before15s timeout.
This is better exit behavior than R13, but the large climb and recovery
transient remain real limitations, not a perfect transition.

Exactly one postback braking activation appears. No repeated Land braking
occurs. Later MC RTL return is commanded translation, not failed stationary
hover. After return, the craft descends, detects landed and disarms about1.53s
later. No new accel-bias or compass warning appears in the recorded messages.
Without raw truth this does not revalidate the sensor pipeline at all FPS.

## R15: Bounded Candidate Changes

| Parameter | R14 | R15 | Scope |
| --- | ---: | ---: | --- |
| FW_RR_P |0.18|0.22|Further22.2% P-only FW damping trial |
| MPC_XY_CRUISE |3.5|5.0|Default autonomous horizontal request |
| MPC_XY_VEL_MAX |4.0|6.0|Horizontal bound; also braking activation bound |
| MPC_VEL_MANUAL |4.0|5.0|Manual-position speed, not a hover-speed target |
| MPC_Z_V_AUTO_UP |2.0|2.5|Autonomous climb request |
| MPC_Z_VEL_MAX_UP |2.4|3.0|Paired climb bound |
| MPC_Z_V_AUTO_DN |1.4|1.6|Autonomous high-altitude descent |
| MPC_Z_VEL_MAX_DN |1.4|1.8|Paired manual/vertical bound |
| MPC_TKO_SPEED |1.2|1.5|Initial takeoff climb, same2.5s ramp |

A stationary hover still targets zero velocity. Increasing travel speed is
not increasing hover RPM or hover thrust. Keep MPC_THR_HOVER0.26 and estimator
enabled. Near-ground MPC_LAND_SPEED0.9, crawl0.36 and landing heights8/3/1m
remain unchanged. Faster descent at altitude must not also increase touchdown
speed without separate evidence.

All MC PID, acceleration, jerk, tilt, FW I/D/FF, differential scales, TECS,
airspeeds, NPFG, bank/radius, geometry/CG/inertia and transition timing stay
fixed. Firmware is identical to R14. R15 has90 explicit profile parameters.

Important coupling: PX4 uses1.3*maximum-speed bounds for emergency braking.
Horizontal activation changes5.2->7.8m/s; upward3.12->3.9m/s; downward changes
2.184->2.496m/s using max(MAX_DN,1.2*AUTO_DN). This is NOT an independent
braking disable or a hidden fix. Prior9.2m/s postback speed would still exceed
the new horizontal threshold, but different physical entry conditions can
change activation and settling. Compare actual entry speed and diagnostic
events in the next log.

A fixed deceleration limit requires longer stopping distance at higher speed:
raising cruise3.5->5 means roughly double kinetic-energy/stopping-distance
demand under the same ideal constant deceleration. Leave room at each target.
These are user-requested candidate speeds, not measured maximum capability
or a claim of unchanged stability. Simultaneous MC speed/FW P changes mean
a complete flight is not a single-variable A/B.

Only runner usability also changes: dirty-checkout errors now print the exact
changed paths, without resetting, cleaning, ignoring or deleting user work.
No extra legacy directory or second PX4 clone is introduced.

## What Belongs Where

The original [X-Plane SITL airframe PR22493](https://github.com/PX4/PX4-Autopilot/pull/22493)
is already merged (2026-07-01). The remaining observations do not establish a
transport/frame/sensor bug requiring an official bridge-core change.

1. px4xplane experimental branch: Ref6 ACF/config pairing, explicit tuning,
   evidence workflow and current test guide. Keep the production plugin's
   telemetry/control mapping unchanged unless evidence identifies a defect.
2. PX4 fork validation branch: same official firmware and logging-only commit;
   no new core patch or cherry-pick for these gain/speed changes.
3. Eventual official PX4 follow-up: reviewed airframe defaults only, paired to
   the exact released aircraft. Original5021 describes a5kg aircraft with
   different arms/model; Ref6 is2kg. Do NOT silently transplant all Ref6 gains
   into5021 or other four aircraft and call that universal tuning. Decide the
   supported model/airframe identity explicitly before proposing defaults.
4. Merged core fixes remain upstream. This R15 iteration is not a reason to
   reopen the merged bridge PR, duplicate28350, or update unrelated open PRs.
   PR27670 remains separate; this log does not establish a reason to add it.

## Production Exit Checklist

Successful takeoff/RTL/Land is demonstrated in two runs. Production promotion
still requires:
- Complete, monotonic new truth capture; clean source/model/profile identities.
- Sustained FW tracking without growing or persistent unacceptable roll mode.
- Faster MC GoTo, stop, climb, descent, back-transition and Land tracking.
- Quantified back-transition climb/recovery, not just a successful MC label.
- Resolve FW_R_TC1.5 outside metadata max1.0. It is active at runtime, not
  silently clamped. Normalizing it to1.0 raises outer-loop gain50%, so do not
  bundle that with inner-loop damping or declare the current value production.
- Measure trim, minimum/stall behavior, maximum sustainable speed and climb/sink
  performance separately. Current17/21/26/34m/s are provisional seeds, not
  measured or certified envelope limits.
- Standardize final model/airframe identity, then regression-test all five
  existing production airframes before public commit/tag/release promotion.
- Cleanly separate public user installation from optional developer logging/
  patches. Disable private upload by default in production; remove temporary
  debug intake only after evidence delivery/review and explicit closeout.

Do not relabel R15 as a final production release before those checks pass.

## Next Test

Follow QUICKSTART.md in the SAME folder. Fresh flight FIRST, then NEW recording;
never reload/reset during the captured run. Console takeoff180m, one MC GoTo/
stop and one small altitude up/down pair, then a clean75s FW segment without
retargeting, normal back-transition and Land. No new code rebuild is required
beyond the runner's normal pinned build check. Watch the new speed settings,
FW mode amplitude, actual recovery and descent. Keep the airborne run under
eight minutes with a fully charged simulator battery.

No additional public PR or plugin release is made yet. Reports/plots and the
checksum-verified debug update are delivered to Telegram. The two unavailable
reviewers are not represented as completed expert approval.

Sources: [PX4 jerk-limited trajectories](https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory),
[PX4 FW performance tuning](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing),
[QGC Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html).
Pinned source is58794523a1: FlightTaskAuto.cpp braking/landing constraints,
MulticopterPositionControl.cpp paired-limit enforcement and official5021 defaults.
