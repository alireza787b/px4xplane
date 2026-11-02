# QuadTailsitter R8 Failure Review And R9 Recommendation

Date: 2026-11-02. Experimental SITL only; not a hardware tune or release approval.

## Bottom Line

The best-supported immediate correction is faster MC roll-rate tracking:
MC_ROLLRATE_P 0.031815 -> 0.060, with I/D/K unchanged. Validate it with heading-
held sideways GoTos, then Land. Do not weaken emergency braking or change EKF
limits to hide the symptoms.

This is NOT a complete fix for FW/back-transition. They remain failed validation
items with separate dynamics. A successful stationary hover did not establish
that the roll axis could reject the large disturbance after back-transition.

## Evidence Identity

- Uploaded ZIP: ed53038d2fdbee81049c0be0ace425dd395ec338a2256934998f5be1d8690863.
- Runner: 2026-11-02.ref6.8; ULog 2026-11-02/08_38_55.ulg.
- PX4 diagnostic pin: 9cfe130a8d8e0c25f88e044141c523247ca53d48.
- Official base: f90b40d61e56405513dfcd8e194287ac60d3a6ae.
- Loaded-path ACF hash: c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
- Actual simulator mass 2.000000477 kg, zero payload and wind. No ULog dropouts.
- This run ended airborne/armed. It is not a landing pass.

## Recorded Sequence

All times below are PX4 boot seconds, not video or wall-clock time.

| Event | Time | Evidence |
| --- | ---: | --- |
| Front transition begins | 112.906 | VTOL state 1 |
| FW begins | 126.186 | VTOL state 4 |
| Normal back-transition begins | 163.355 | VTOL state 2, not quad-chute |
| MC mode begins | 178.368 | Timer exit, not pitch criterion |
| Land begins | 211.828 | nav_state 18; accepted |
| End of log | 381.530 status sample | Still no landing/disarm |

The QGC takeoff target was approximately 104.5 m above home, not the requested
180 m console scenario. FW lasted 37.17 s, not five. Those differences are
recorded, but do not excuse the failed control/recovery.

## Three Distinct Problems

### 1. FW Roll Is Not Properly Damped

The correctly transformed FW bank varies approximately -14.2 to +29.0 degrees.
Physical MC yaw is the FW roll axis: its rate oscillation peaks around 0.4 Hz,
with approximately 0.378 rad/s tracking-error RMS in the established FW window.

The disturbance starts during front transition, before the FW rate controller
takes over. Thus the new FW gain coefficients alone cannot explain its onset.
Constant normalized MC/FW coefficients do not guarantee equal physical response:
collective, prop inflow, Q/T, attitude-loop gains and filtering differ.

The R8 source-frame/normalized-gain contract is correct, but it was not a flight
stability guarantee. A simple local FW model fits much worse than the MC roll
model; it does not justify claiming an optimal FW gain from this recording.
Cant-vector/actual prop-force correspondence is still an open physical-model
validation item. No motor orientation/sign reversal or new aircraft geometry
is justified by this log.

### 2. Back-Transition Did Not Recover By Its Exit Criterion

It ends at 178.368 s because the 15 s timer expired. Logged exit pitch is
-0.92157 rad (-52.8 degrees), versus an angle criterion around -14.9 degrees.

MC attitude/rate control was already active DURING back-transition, from
163.355 s. There is not a new MC rate-controller start at the final mode label.
The final label releases transition attitude guidance into ordinary MC guidance
while the vehicle is still tilted and moving rapidly.

The second-by-second CSV includes requested/actual pitch, pitch rate, torque,
motor range, speed and altitude. For example:

| Boot s | MC pitch actual / requested deg | Pitch rate actual / requested rad/s | Horizontal speed m/s |
| --- | --- | --- | ---: |
| 165 | -80.6 / -76.4 | -0.012 / +0.126 | 33.1 |
| 170 | -66.4 / -46.4 | +0.116 / +0.205 | 23.3 |
| 175 | -54.0 / -16.4 | -0.159 / +0.375 | 10.4 |
| 178 | -51.9 / -0.5 | +0.023 / +0.503 | 12.0 |
| 181 | +12.0 / +27.3 | +0.551 / +0.215 | 6.3 |

It climbs roughly 45 m during recovery. Pitch demand exists; the aircraft is
not simply being told to remain in a dive. Increasing the duration again alone
is not a demonstrated solution. FW roll damping and pitch authority through
changing inflow/collective need to be separated before accepting recovery.

### 3. MC Lateral Oscillation Repeatedly Interrupts Land

Late MC roll is roughly +/-30 degrees, with setpoint repeatedly reaching the
30-degree tilt limit, at about 0.15-0.2 Hz. Roll-rate error is about 0.21 rad/s
RMS and closed-loop correlation lag approximately 0.48 s. Pitch-rate error in
the Land window is much smaller, approximately 0.047 rad/s.

This is sustained oscillation/limit-cycle behavior, not a critically damped
response. It is not just a visual roll reading or an unused elevator issue.

Diagnostics record 33 emergency-braking activations, all horizontal-speed reason.
The trigger checks trajectory speed above 1.3 * MPC_XY_VEL_MAX = 5.2 m/s.
Braking reseeds the trajectory from measured position/velocity and requests
all-axis stopping. Release requires the trajectory to slow sufficiently.

During Land, raw requested descent stays +1.4 m/s downward, but braking is active
in 70.2% of diagnostic samples. Mean measured descent while active is -0.0094 m/s
(almost zero), versus +0.375 m/s outside braking. Therefore Land WAS accepted;
its descent is repeatedly interrupted by the lateral-control/braking cycle.
This does not prove whether braking initially starts or only sustains that cycle.

The braking behavior exists in official f90, not a private behavioral addition
inside diagnostic commit 9cfe. No evidence of transferred FW integral, in-flight
estimator reset or minutes-long stale filter state was found. MC roll integral
is small and motors have headroom; upper motor clipping does not explain the
persistent late-MC motion.

## Estimator And Plugin Checks

- Quaternion disagreement with logged groundtruth: 0.567 degrees RMS.
- Angular-rate disagreement: about 0.0073 rad/s RMS per axis.
- Accel-bias component RMS approximately 0.0072 / 0.0138 / 0.0098 m/s2.
- Independently aligned CSV vertical-velocity disagreement: 0.0157 m/s RMS.
- Constant truth-minus-boot offset 222.11369 s; interior altitude-fit RMS 0.0028 m.
- Actual mapped ENGN_thro_use channels track motor commands: about 0.0023 RMS.

The CSV ends before the final ULog samples: comparisons use overlapping coverage,
not extrapolated truth. Cockpit throttle_ratio and ENGN_thro are zero while
ENGN_thro_use changes: only the actual configured actuator dataref is meaningful
for this mapping check. A frozen cockpit display must not be treated as a failed
motor command.

Groundtruth topics share the plugin's transform, so agreement alone cannot prove
every sensor path correct. These independent checks do not show an EKF/pipeline
failure sufficient to explain this recording. No production plugin or PX4-core
change is supported by the present evidence.

## Why This Gain, Not Another Broad Retune

Four 25 s MC windows give consistent local roll-acceleration/torque models,
roughly 64-67 rad/s2 per normalized torque with an apparent 0.12 s fitted delay.
Smoothed acceleration fit R2 is 0.93-0.97. Delay partly depends on smoothing.

Fixed recorded-rate-demand replays predict:

| MC roll P | Replay rate-error RMS across four windows, rad/s |
| ---: | --- |
| 0.031815 | 0.222-0.239 |
| 0.040 | 0.186-0.199 |
| 0.060 | 0.135-0.142 |
| 0.080 | 0.105-0.110 |

Choose 0.060 for an accelerated SITL trial: a substantial measurable step, rather
than changing all loops at once. The larger 0.080 candidate can remain a later
option. These are IN-SAMPLE local rate-model screens, not full X-Plane simulation,
open-loop identification, closed-position-loop stability or a flight guarantee.
The model omits changing aerodynamics, allocation limits and position/braking
feedback. Measured next-flight results decide.

An independent Astra agent checked the control-state/landing chain and the
bounded tuning choice, including the fixed-yaw enum. This is an agent review,
not human PX4-maintainer approval.

## R9 Changes And Next Test

- MC_ROLLRATE_P becomes 0.060. MC roll I 0.007954, D 0.000636 and K 1 unchanged.
- Every FW coefficient is frozen at R8; particularly FW_YR_P remains 0.099422.
  No automatic MC-to-FW gain propagation during this isolated test.
- No ACF, mass/CG, propulsion, rotor mapping, EKF, speed or braking-limit change.
- Existing hover scenario adds checked MPC_YAW_MODE=5 for heading-held lateral
  GoTos; 4 would turn toward the waypoint first. The override is separately
  recorded and isolated to this run. Base gain profile is not given a new yaw mode.
- No new PX4 PR, main-branch tuning, hardware firmware or production release.

Run the same-folder update and hover command in QUICKSTART.md. Console takeoff
to 50 m, settle, 10-15 m sideways GoTo, settle, return, settle, then Land at current
position. This is a short test of the previously under-exercised axis, not a
request for another long uncontrolled full flight. Shutdown automatically
captures truth and uploads the evidence.

Following the MC result, revise FW roll separately using actual differential
torque/response at cruise, then verify pitch recovery across back-transition.
Do not approve another timer-only recovery as success. Reconsider rotor cant,
thrust linearization or geometry only if force/moment evidence supports them.

## Reproduce And References

tools/analyze_qtailsitter_handoff.py ZIP NEW_OUTPUT_DIR requires numpy/pyulog.
It writes frame-correct FW/recovery/Land/braking plots, metrics, a one-second
back-transition table and local gain screens without extracting the large ZIP.
The output must be a new directory; raw captures remain preserved.

- [PX4 MC PID tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter): tune the inner rate loop before outer attitude/position loops.
- [Exact official braking logic](https://github.com/PX4/PX4-Autopilot/blob/f90b40d61e56405513dfcd8e194287ac60d3a6ae/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp): _checkEmergencyBraking, all-axis stopping and raw Land setpoints.
- [Exact yaw-mode enum](https://github.com/PX4/PX4-Autopilot/blob/f90b40d61e56405513dfcd8e194287ac60d3a6ae/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.hpp): yaw_fixed=5.
- [Exact tailsitter transition](https://github.com/PX4/PX4-Autopilot/blob/f90b40d61e56405513dfcd8e194287ac60d3a6ae/src/modules/vtol_att_control/tailsitter.cpp): MC control during transitions and angle-or-timer exit.
