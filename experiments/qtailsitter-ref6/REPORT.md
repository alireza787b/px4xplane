# R12: What R11 Actually Demonstrated

Experimental SITL only. No claim of perfect tuning, production readiness or
hardware suitability. R12 is a controlled recovery experiment plus better
measurements, not another blanket PID/aircraft redesign.

## Findings

- R11 failed full-flight recovery. The failure is real and precedes manual edits.
- Initial MC hover is quiet; the large oscillation appears in FW and after back.
- Back-transition now exits by the actual pitch-angle criterion, not its timer.
  It still ends with about 9.3 m/s horizontal velocity, so recovery is not done.
- MC recovery repeatedly leaves emergency braking with acceleration still stored
  in the trajectory. Very low normal jerk cannot unwind it promptly; the virtual
  trajectory crosses the braking trigger again. Braking interrupts Land descent.
- FW roll oscillation is a separate, earlier problem. MC trajectory jerk will
  not directly fix it.
- No current evidence supports changing mass, adding ballast, reversing motors
  or masking estimator checks.
- Near 800 s, commands saturate while available shaft power falls. This is a
  separate late event strongly consistent with battery depletion/derating.

The prior scalar attitude-loop screens were not adequate to predict this coupled,
switching guidance/aircraft response. Their apparent settling improvements did
not establish full-loop stability. We should not repeat that tuning method as
a substitute for identifying the active mechanism.

## Provenance And Timeline

Archive SHA256:
`bcbd3657db08a5382a209b08fa3aa85044ae6e868004307bd54fefe279e51814`.
ULog: `2026-11-02/19_15_01.ulg`. Runner: `2026-11-02.ref6.11`.
Actual plugin log identifies v4.2.1. The validation branch contains older plugin
source; plugin behaviour was checked against tag v4.2.1, not that older source.
PX4 diagnostic pin: `9cfe130a8d8e0c25f88e044141c523247ca53d48`.
Official base: `f90b40d61e56405513dfcd8e194287ac60d3a6ae`.
ACF SHA256: `c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918`.

| Event | PX4 boot seconds |
| --- | ---: |
| Front transition | 68.007675 |
| FW established | 80.711455 |
| Back-transition | 109.539837 |
| MC angle-based exit | 124.438355 |
| First horizontal emergency braking | 124.446136 |
| Land requested | 162.452477 |
| Hold requested | 195.347638 |
| FIRST manual gain change | 221.032061 |
| FW autotune attempts, while in MC Hold | 393.231413 / 477.166577 |
| Motor commands first exceed 0.95 | about 795.50 |

There are no ULog dropouts. The complete 97,184-row truth window is present:
388,733,193 raw CSV bytes compressed to 61,816,238 bytes. R11's compression and
upload repair worked. Disk identities do not prove simulator-memory identity.

## Recovery: A Specific Chain, Not A Generic PID Guess

Before transition, 55-67 s horizontal speed RMS is 0.0148 m/s and attitude-error
RMS 0.140 degrees. That is good small-disturbance hover, not demonstrated
large-disturbance recovery.

Back exits at -14.43 degrees pitch after 14.8905 s, just inside the -0.26 rad
criterion. It is angle-based, unlike R10's timer exit at about -59.3 degrees.
Pitch-rate error during back is still 0.439 rad/s RMS and some motors briefly
reach zero; it is not a perfect transition.

First braking release is at 130.166620 s. The smoother's retained horizontal
acceleration is [4.150,-1.281] m/s2. Subsequent pre-edit releases average about
4.83 m/s2. Normal settings are only MPC_ACC_HOR=0.7 and MPC_JERK_AUTO=0.7.
Changing constraints does not reset existing acceleration.

The virtual trajectory then repeatedly crosses 1.3*4=5.2 m/s and reactivates
braking. These are virtual-state triggers, not necessarily actual overspeeds:
one reactivation latched virtual velocity [5.047,-1.323] m/s while the first
diagnostic sample measured [2.047,-0.682]. During braking, position/velocity are
reseeded from measurements, but acceleration remains a trajectory state.

Finite attitude response, +/-30-degree tilt clipping, and repeated guidance
switching sustain a roughly 0.145 Hz postback pitch cycle. Land-late rates lag
requests about 12.5 degrees at that frequency, versus 56.6 degrees attitude lag.
Small normalized torque and lack of allocation saturation do not prove a perfect
plant, but do not support an early high-power/weight shortage.

During Land (162.45-195.35 s), raw down-speed request stays 1.4 m/s. Braking is
active about 55% of diagnostic samples, with nine new activations. Downstream
request averages 0.209 m/s and actual descent 0.190 m/s. The cycle continues
after switching back to Hold. Land was accepted; it was not refusing descent
because the vehicle had not become stationary. The ground was never reached.

## Real PX4 Smoother Replay

The new host tool compiles the pinned, unmodified VelocitySmoothing.cpp/.hpp,
initializes its three axes from 25 recorded pre-edit release samples and asks
for a fixed zero-velocity stop. It keeps normal acceleration/speed limits and
synchronizes XYZ. This is NOT an aircraft simulation or an exact guidance replay.

| Normal jerk m/s3 | Peak virtual XY speed range m/s | Releases crossing 5.2 m/s |
| --- | ---: | ---: |
| 0.7 | 7.27-14.22 | 25/25 |
| 2 | 2.42-4.60 | 0/25 |
| 4 | 1.21-2.01 | 0/25 |
| 8 | 0.57-0.86 | 0/25 |

This isolates a genuine acceleration/jerk state-handoff mechanism. It does not
establish how the complete aircraft will respond. Recorded samples are at the
diagnostic publication rate, not exact switch instants; the screen uses a fixed
stop target, time-stretch=1 and no repeated braking or aircraft feedback.

MPC_JERK_AUTO=4 is the pinned PX4 default, not an invented speed override.
Our inherited 0.7 jerk is below its metadata minimum 1; MPC_ACC_HOR=0.7 is also
below its metadata minimum 2. Small positive values can still execute, but are
not an automatically safer or smoother recovery configuration. R12 changes
only jerk in the selected case, so the remaining low acceleration is explicit.

Reproduce with an existing PX4 checkout and g++:
`python3 tools/replay_qtailsitter_braking.py braking_preedit.json --output replay.json`.
In the delivered bundle use:
`python3 evidence/replay_qtailsitter_braking.py evidence/braking_preedit.json --output replay.json`.

## FW Is Still Unresolved

FW roll is physical negative MC yaw, not MC roll. In 82-109 s it oscillates near
0.448 Hz with rate harmonic amplitude 0.231 rad/s and bank -8.22 to +22.35 degrees.
Motors remain about 0.077-0.290, without reported allocation deficit. Rate error
is about 0.20-0.22 rad/s RMS in the reviewed windows. Different routes/windows
prevent a causal R10/R11 performance ranking; lack of damping is the failure.

Observed thrust and reaction-torque changes are not simply too slow at this
frequency: differential thrust lags command about 15 degrees, and the constructive
cant-plus-drive-torque model about 9.5 degrees. Rotor-acceleration reaction is
mainly in quadrature and does not establish cancellation. Physical cant sign,
total prop moment, aerodynamic moments and body inertia remain unmeasured.
If actual cant opposes reaction torque they could partially cancel; this is a
hypothesis to test, not justification to flip AX/AY/KM blindly.

R12 adds native total/prop/aerodynamic L/M/N moments, forces and runtime
unit-mass inertia to distinguish incorrect effectiveness from aerodynamic
coupling. No new FW gain or geometry is claimed justified yet.

## Mass, Power And Estimator Checks

Actual mass is constant 2.000000477 kg, payload/fuel zero, CG-offset datarefs zero.
Early hover thrust is about 19.44 N versus weight 19.61 N, at about 5,900 RPM and
278 W combined shaft power. FW median thrust is about 5.69 N with wing lift
supporting weight; a low scalar FW T/W is not proof of insufficient lift.

Before 790 s no motor command exceeds 0.423. Observed max prop RPM is below
9,400 versus a 13,000 limit. Full-command thrust capacity was NOT tested, so do
not claim a measured design T/W. No full body-inertia measurement was available.

Independent quaternion differentiation matches P/Q/R, correlations >0.999996.
Truth/ULog clock alignment using quaternion, rates and velocity agrees:
sim_time approximately 41.19685 + 1.00006055*boot_time. The drift is tens of
milliseconds across the run, not seconds of backlog. Estimated rates differ
from independent truth by about 0.007 rad/s per axis and pre-edit velocity by
about 0.01-0.03 m/s. Normalized PWM maps to actual throttle with slope near one.
These checks argue against a sensor/axis/channel-transfer cause in this recording,
not a blanket certification of every bridge path or FPS.

## Separate Late Energy Event

ACF battery capacity is 66.6 Wh and modeled drive efficiency about 0.90.
By 799 s, integrated shaft energy is about 59.09 Wh, equivalent to 65.65 Wh at
that efficiency, before auxiliary loads. Initial charge was not recorded.

At 780 s commands are about 0.27 with 272 W shaft power. Near 795 s commands
rise toward saturation but power falls to about 196 W, then 159 W near 799 s.
This strongly suggests exhaustion/derating; battery charge/voltage/current
are added to verify it. It cannot explain the early Land failure: energy at
200 s is only about 13.36 Wh. Do not leave a failed recovery hovering for minutes.

## Why Slider Changes Did Not Settle It

All 111 genuine in-flight changes occurred in MC Hold, after the original failure.
There were no poststartup FW PID changes, so they were not a FW-roll tuning test.
The edited MC rate/attitude/position gains affect different loops and changed
together; they do not remove the measured retained-acceleration/retrigger chain.
They also do not prove that PID tuning is irrelevant.

Both autotune attempts timed out without a usable gain result. They ran in MC,
not established FW. Shared autotune status can add rate excitation to the active
MC controller; those later windows are not clean plant-identification data.
Full timestamp/old/new/mode records are retained in the evidence.

## Code History And Upstream Claims

The pinned base includes PR28170, merged as
`185ce66e5b2b49e739df1f9f20a314a52027d25d` on 2026-08-21. It reseeds measured
position/velocity while braking and changes the release threshold from 0.01 to
1 m/s to avoid a noise-latched brake. The reviewed diff does not reset acceleration.
That establishes relevant code history, NOT that it alone caused all our failures.

Issue18440 discussed high-speed post-Quadchute trajectory recovery. PR18614
proposed horizontal emergency braking and velocity/acceleration-based exit, but
GitHub marks it CLOSED, not merged; its issue closure must not be described as
proof that that exact PR merged. This history motivates the diagnostic; it is
not an identical reproduction of the present failure.

No new PX4 control fix, upstream PR or plugin binary change was made for R12.
If the isolated tests establish a general release-state problem, a future fix
must preserve measured-state reseeding, avoid noise latching, respect jerk
continuity and test all axes/modes. Raising a speed threshold is not that fix.

## Delivered R12 Changes And Next Test

Baseline aircraft, PID and firmware stay unchanged. Named cases are separate:
- recovery-jerk: ONLY MPC_JERK_AUTO=4, braking threshold stays 5.2 m/s.
- braking-isolation: ONLY MPC_XY_VEL_MAX=12, jerk stays 0.7, cruise stays 3.5.
  This also suppresses initial braking near 9.3 m/s; not a pure switch or fix.
- baseline: unchanged R11, retained for reproducibility, not the recommended run.

Do recovery-jerk first, using QUICKSTART.md. A second case is available only if
the first result needs further isolation. No simultaneous cases, gain edits,
autotune, GoTo or RTL. New datarefs must resolve before checkout/build launch.
prepare-truth/restore-truth preserve the custom recorder catalog under a receipt.
Existing aircraft/config, credentials and runs are not replaced by the update.

Acceptance is measured: damped MC recovery, no repeated braking, uninterrupted
Land request/response, touchdown/disarm, plus independent FW moment diagnosis.
No R12 X-Plane flight has been performed on this server. Host regression and
smoother replay cannot replace that test. Independent AI control/physics audit
artifacts were cross-checked here; this is not human maintainer approval.

## Host Verification

- 32 repository tests, 127 runner/recorder/updater tests and six upload-server
  tests passed. This includes compiling the real pinned smoother for the
  release-state regression; all 25 recorded states were also replayed separately.
- Independent code review caught and fixed recorder-journal symlink handling
  and a resumed-prepare installation-state check. Both have regressions.
- Missing diagnostic references reject either new case before a run directory,
  source checkout change, build or simulator launch.
- Generated ACF identity and reusable PID profile remain unchanged from R11.
  There is no new control patch, plugin binary, public release or hardware image.
- Package integrity and installed R11-to-R12 update compatibility are verified
  separately in the delivery handoff. Host tests are not X-Plane flight tests.

## Sources

- [Pinned braking change, PR28170](https://github.com/PX4/PX4-Autopilot/pull/28170)
- [Earlier high-speed recovery issue18440](https://github.com/PX4/PX4-Autopilot/issues/18440)
- [Closed horizontal-braking proposal18614](https://github.com/PX4/PX4-Autopilot/pull/18614)
- [Pinned trajectory parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/f90b40d61e56405513dfcd8e194287ac60d3a6ae/src/modules/mc_pos_control/multicopter_autonomous_params.yaml)
- [PX4 MC tuning sequence](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter)
- [PX4 jerk-limited trajectory](https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory)
- [Laminar dataref catalog](https://developer.x-plane.com/datarefs/)
- [TruthCapture custom datarefs](https://github.com/alireza787b/xplane-truth-capture#custom-datarefs)
