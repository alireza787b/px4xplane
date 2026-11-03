# R15 Flight Review And R16 Tuning Plan

2026-11-03. Experimental QuadTailsitterRef6 SITL only. R15 completed takeoff,
FW, RTL, back-transition, descent, touchdown and automatic disarm. Residual
oscillations and recovery overshoot remain; this is not production acceptance.

## Evidence Integrity

- Archive: a56e070a5e674501bc6d59fe49a1f42d570e2a7b0dd6404b760f17cedc36c860.zip,
  81,926,878 bytes. ZIP CRCs and all30 file hashes pass.
- ULog: 2026-11-03/11_30_48.ulg, 112,008,237 bytes, no recorded dropouts.
- All90 R15 profile values match startup. No in-flight parameter changes.
- Complete independent TruthCapture: 51,617 rows, frames5510..57126,
  simulator time69.428..506.655s. No rejected/reset segment in this upload.
- PX4 official base649929706fb34bf0b06dbd2df24e04d5d452d616 plus logging-only
  child58794523a151fa33a28a564454454b8c9fbfcdd5. No private dive/optional patch.
- ACF remains c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
  Truth confirms actual2.0000005kg; geometry, motor order/signs and power caps
  are unchanged. No aircraft replacement is needed for R16.

Truth was aligned against bridge ground-truth rates, not against EKF attitude.
Fitted simulator-minus-boot offset is71.9869s; separate windows differ by
about17ms. Rate matching RMS is0.000634rad/s over105..290s. Estimated N/E/D
velocity versus independent truth RMS is0.0199/0.0165/0.0139m/s over30..418s.
These include sampling/alignment effects. This run does not support a gross
axis/sensor/vertical-velocity pipeline error, but does not prove every FPS,
airport, platform, aircraft or weather condition.

No airborne accel-bias, compass or velocity warning is recorded. Peak estimated
accel-bias vector norm is below0.08m/s^2. Startup briefly reports no power and
no heading reference, then Ready at5.159s. Do not confuse startup convergence
with a sustained airborne estimator failure.

## Actual Scenario

Times are PX4 boot seconds. QGC requested explicit takeoff altitude1573.65mMSL,
about82m above home, overriding the180m console default. No pre-FW GoTo or
altitude up/down pair is recorded. The later MC RTL leg does test5m/s travel,
but not the requested out-and-back stop maneuver.

| Event | Time s | Result |
| --- | ---: | --- |
| Takeoff / hold |26.595 /65.447|Steady MC climb and hover |
| Front / FW |105.068 /117.866|Normal front-transition |
| RTL |188.368|Commands return at180m above destination |
| Back / MC |258.217 /273.229|15s timeout, not angle completion |
| RTL Land target |314.106|Descent inside RTL |
| Touchdown / disarm |420.892 /422.410|Automatic disarm1.52s later |

Land lasted about107s from roughly163m above local origin. Its duration is
consistent with the commanded descent, not a rejected/stuck Land command.

## Measured Improvements And Remaining Defects

| Item | R15 measurement | Interpretation |
| --- | --- | --- |
| MC climb38..55s |2.495m/s; vertical command-error RMS0.047m/s|New2.5m/s request tracked |
| Quiet hover70..80s |Height spread0.051m; XY speed mean0.024m/s|Initial MC remains well controlled |
| MC RTL return298..314s |XY speed4.985m/s|New5m/s travel request tracked |
| High Land324..390s |Down1.600m/s; vertical error RMS0.052m/s|Continuous descent, no repeated braking |
| FW145..165s |Roll mode0.443Hz, bank amplitude0.72deg|Smaller residual mode, not zero |
| FW165..185s |Roll mode0.450Hz, bank amplitude1.05deg|No large sustained cruise wobble here |
| RTL200..258s |Roll mode0.461Hz, bank amplitude2.22deg|Operating-condition sensitivity remains |
| RTL energy response |EAS about24.6..30.9m/s for26target; actual climb rate about-4..+4m/s|Pitch/throttle energy oscillation, not only lateral guidance |
| Back-transition |Height gain38.83m, max climb about6.8m/s, exit pitch-18.0deg|Successful mode change is not a clean transition |
| Postback |Pitch peak about+49deg, decays rather than persisting|Late rotation/reversal needs improvement |

The R14 longer FW record had approximately5deg bank-mode amplitude, versus
about0.7..1.1deg in R15 cruise. Different commands/entry conditions prevent
claiming an isolated causal A/B or a universal percentage improvement.

Native moments confirm the propulsive contribution opposes the FW roll mode;
the aerodynamic contribution partly offsets it. Gain compression stays above
0.9918, so large automatic gain suppression is not the primary explanation.
These closed-loop observations are not identified stability margins.
Near vertical flight, MC Euler roll is ill-conditioned; the large roll numbers
at the start of the MC plot are not independently proof of a physical bank
departure. Recovery analysis also uses body rates and quaternion rotation error.

Back-transition is a distinct pitch problem: around271..272s, pitch-rate
request reaches roughly50deg/s while measured rate is about4deg/s. At273s it
rotates rapidly, then overshoots before recovering. Rate integral reaches
about0.081, significant relative to the P effort even below its limit.
Opposing aerodynamic moment is also substantial; allocation success alone
does not prove achieved physical torque or make this purely a PID defect.

Motor allocation torque residual remains below5e-8 during back/recovery;
back motor range is roughly0..0.30. Only brief floor clipping appears, not
sustained maximum-thrust saturation. Increasing mass/power, changing motor
signs or hiding estimator checks is not supported by this evidence.

Exactly one postback braking activation occurs; no repeated Land braking.
The official upstream braking follow-up[#28350](https://github.com/PX4/PX4-Autopilot/pull/28350)
is already in this firmware. Do not add a duplicate controller workaround.

## R16: Four Candidate Changes

| Parameter | R15 | R16 | Reason and consequence |
| --- | ---: | ---: | --- |
| FW_RR_P |0.22|0.26|Bounded18% inner-rate damping increase; may also amplify noise/new modes |
| MC_PITCHRATE_P |0.12|0.16|33% increase to reduce pitch-rate deficit and oppose overspeed sooner; affects ordinary MC too |
| FW_T_I_GAIN_PIT |0.25|0.10|Reduce energy-balance integral action to pinned default; slower trim convergence is the tradeoff |
| FW_THR_TRIM |0.25|0.18|Partial correction toward measured level-flight throttle; also changes climb/sink feedforward |

R15 steady145..185s mean throttle was0.1774 at26EAS, with near-level flight
and negative throttle integral. PX4 density compensation made the old0.25
parameter approximately0.270 effective trim. New0.18 would be about0.194 at
the same density, not exactly0.1774. This is a conservative local correction,
not a calibrated power/drag map or a reason to force the parameter to0.1643.

All other MC/FW PID terms, attitude gains/time constants, TECS damping/filter,
NPFG, speed bounds, acceleration/jerk/tilt, near-ground Land speed, transition
duration, firmware, ACF/config, mass/CG and mapping remain fixed. Profile now
contains91 explicit parameters. It is a combined trial, not four causal A/Bs.

An independent AI review challenged a wider proposed bundle. As a result:
- Do NOT combine FW_R_TC1.5->1.0 with the new roll P: their proportional
  attitude-error-to-control coefficient would rise about77%, not18%.
- Do NOT also change FW_T_STE_R_TC1.0->0.4; less lag also passes more energy-rate
  variation into throttle feedback/integration. Filter lag alone is unproven.
- Do NOT alter transition timing or geometry to conceal the observed transient.

The review was source/data-based AI analysis, not human PX4 maintainer approval
or a guarantee. R16 must still be flown in X-Plane. No server-side physics
simulation of this model or actual hardware validation is claimed.

## What Changes In px4xplane And PX4

1. Now: only experimental Ref6 profile, tests and current documentation on the
   px4xplane validation branch. Same in-place update, no extra clone/aircraft.
2. No production plugin C++ change, sensor calibration hack, new EKF limit,
   core controller patch, private dive feature or hardware firmware is added.
3. The original X-Plane SITL[PR22493](https://github.com/PX4/PX4-Autopilot/pull/22493)
   is already merged. This flight does not establish a new bridge-core bug or
   justify reopening that PR. Current fixes are airframe-specific tuning.
4. After acceptance, decide one supported model identity and propose a clean
   airframe-default/model-pairing follow-up. Original5021 describes a5kg model
   with different arms; do not silently transplant2kg Ref6 gains into it or
   the other four aircraft.
5. Resolve inherited FW_R_TC1.5 outside metadata maximum1.0 in a separate,
   explicit outer-loop validation. Also review FW_WING_SPAN3m versus actual
   0.6m and wing-height clearance together before production. They are known
   configuration debt, not claimed current high-altitude oscillation fixes.
6. Measure stall/trim/max sustainable speed, full-power climb and idle-sink
   envelope. Current17/21/26/34m/s speed seeds are not measured limits.
7. Regression-test all five production aircraft; keep beginner official-PX4
   setup separate from developer validation. Private upload/logging must not
   become a default production dependency. Then clean release/version/docs
   and propose only evidence-supported upstream changes, with user approval.

## Next Acceptance Run

Use QUICKSTART.md in the same folder. Include initial MC travel/stops, a clean
FW window and a small explicit altitude step to exercise the energy loop,
then back-transition and Land. Do not call another landing alone a full pass.

Require preserved MC climb/descent, smaller pitch tracking deficit/overshoot,
no worse back climb/exit speed, no larger FW mode, less energy oscillation,
no persistent actuator clipping/new fast mode and complete truth evidence.
If worsening occurs, stop the SITL test; retain the R15 rollback rather than
stacking more unrecorded edits. Production promotion remains pending.

## References

Use the[PX4 FW rate/attitude guide](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing)
and[performance/TECS guide](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing)
as tuning methodology, not aircraft-specific gain guarantees. The pinned
source and parameter metadata take precedence over generic example values.
Relevant pinned files: TECS.cpp, fw_lat_long_params.yaml,
FixedwingRateControl.cpp, FixedwingAttitudeControl.cpp, PerformanceModel.cpp,
tailsitter.cpp, rate_control.cpp and the official5021 airframe file.
