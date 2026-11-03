# R13 Flight Review And R14 Damping Test

Experimental SITL only. R13 landed, but FW tracking and back-transition are
not production-quality. R14 is a one-parameter experiment, not a perfect-flight
claim, new PX4 core fix, public release or hardware tune.

## Evidence Integrity

Automatic retry succeeded. The complete ZIP was on the private upload server;
a separate manual home-folder copy was truncated. No repeat flight is needed.

- Run: flight-tys1du5m; ULog: 2026-11-03/05_30_46.ulg.
- Archive: 43,951,488 bytes; all ZIP CRCs and 30 SHA256SUMS entries pass.
- SHA256: 72b1eb38f78430ffb7f59966ecf2a132f33b43300d0c7c328148e3b191a326e8.
- ULog and 28,777 truth rows present; no dropouts or in-flight parameter edits.
- All 82 applied profile parameters match after replaying startup changes.
  The initial ULog header alone is not the applied profile.
- Firmware: 58794523a151fa33a28a564454454b8c9fbfcdd5, official snapshot
  649929706fb34bf0b06dbd2df24e04d5d452d616 plus logging only.
- ACF hash: c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.
  Recorder-reported loaded path and copied disk file agree; this is not an
  independent simulator-memory hash.

Independent groundtruth-rate alignment gives truth simulator time approximately
PX4 boot time +344.10958 s, with 0.000285 rad/s RMS residual. Window offsets
differ by milliseconds. Estimated NED velocity agrees with truth at RMS
0.0158/0.0195/0.0122 m/s; estimated rates at about 0.008 rad/s.
No accel-bias, compass or unstable-velocity warning follows initial heading
alignment. This flight does not implicate a large EKF/pipeline error; it does
not prove correctness at every FPS, weather condition or platform.

## Timeline

Times are PX4 boot seconds; NED vertical speed is positive down.

| Event | Time s | Observation |
| --- | ---: | --- |
| Takeoff request | 36.701 | External QGC command with explicit MSL altitude |
| Airborne | 39.657 | Land detector leaves landed state |
| Hold | 75.900 | Actual target about 57 m above home |
| Front transition | 81.638 | Normal pitch program |
| FW | 94.276 | Persistent lateral oscillation |
| Back-transition | 118.970 | Deceleration and altitude gain |
| MC | 133.983 | Timeout-only exit, not pitch completion |
| First inactive braking sample | 138.416 | No subsequent retrigger |
| Land | 157.855 | Stops translation and descends |
| Ground contact / landed | 224.842 / 225.560 | Touchdown detected |
| Auto-disarm | 227.087 | 1.527 s after landed |

MIS_TAKEOFF_ALT really was 180 m. QGC supplied 1548.403 m MSL, about 57 m above
home, overriding the default. This is not an altitude-controller failure.
Use console takeoff for the next comparison.

## MC And Landing: Preserve The Working Baseline

MC climb tracks about 2 m/s. The few seconds of settled initial hover have
attitude-error RMS about 0.40 deg and horizontal speed RMS about 0.039 m/s.
This is not an extended MC-envelope or disturbance test.

There is one postback braking activation at approximately 9.29 m/s. It releases
through the no-progress fallback, not a fully stopped condition: first inactive
sample has XY speed 3.09 m/s. The virtual stop precedes the physical stop.
Pitch reaches 41.5 deg during the decaying transient; attitude error remains
below 2 deg from about 153.28 s, 19.3 s after MC entry.

Later 3.1-3.3 m/s motion is commanded: Hold retains the distant waypoint and
requests a 3.5 m/s return. This is not failed stationary hover. Land commands
a stop at the landing location: XY speed stays below 0.2 m/s about 5.1 s later.

Stable Land descent averages 1.402 m/s with vertical tracking-error RMS about
0.034 m/s. Independent truth gives 1.406 m/s over 165-190 s. Descent slows near
ground. There are ZERO Land braking activations. Late landing attitude-error
RMS is about 0.133 deg and XY position-error RMS about 0.149 m. Auto-disarm
timing matches COM_DISARM_LAND=1.5.

This improves substantially on old R11 repeated braking and ineffective
approximately 0.19 m/s descent. It is not a causal A/B: firmware, trajectory
settings and entry conditions changed together. The final
[merged braking fix #28350](https://github.com/PX4/PX4-Autopilot/commit/2750a9bd274476443548d8ed3efdfbd6fa8fd5b4)
is present; no duplicate patch is needed. No new MC gains, braking threshold or
descent-rate changes are justified in R14.

## Back-Transition: Remaining Limitation

Height rises from 58.3 to 95.9 m while slowing from about 29 to 9.3 m/s.
The 37.6 m climb is real. Timeout-only exit occurs at elapsed 15.003 s, pitch
-20.29 deg, short of the -14.90 deg completion threshold. Pitch-rate-error
RMS is about 0.422 rad/s. A brief lower-motor/thrust constraint occurs, but not
sustained maximum-throttle saturation or integral windup.

Kinetic-energy conversion can contribute to the climb; it does not by itself
explain poor pitch-program tracking. Increasing timeout could hide slow recovery
and increase the excursion. Preserve 15 s for comparison, measure the exit and
actual orientation again, and retain this as an acceptance gap.

## FW: Next Tuning Target

The 24.7 s FW interval has a growing mode near 0.449 Hz. Bank ranges -4.67 to
17.22 deg; command ranges -4.76 to 16 deg. Whole-window bank-error RMS is
10.03 deg and roll-rate-error RMS 10.01 deg/s; late rate-error RMS is 11.20.
Modal bank amplitude is 3.96 deg versus command amplitude 0.274 deg. This is
not simply following an equal-amplitude oscillating guidance command.

FW roll rate is NEGATIVE MC yaw rate; FW yaw rate is MC roll rate. Recorded
torque mapping agrees with pinned source. The empty control-surface allocator
reports unachieved axes but is not the motor allocator. Motor commands stay
about 0.082-0.338; motor allocation residuals are numerical noise, roll integral
reaches only 17.5% of its limit, and gain compression stays close to one.

Native moment projections show the prop moment opposes the roll oscillation,
while the aerodynamic rate-aligned component adds modal work of similar
magnitude. P dominates the opposing-rate request. These are closed-loop
finite-window measurements, not independently identified aerodynamic
derivatives, transport delay or stability margins. They support a bounded
P-only damping trial, not arbitrary I/D, cant, inertia or motor-power changes.

Track error falls from about 1166 to 467 m while capturing the 1300 m Hold
orbit. This segment demonstrates neither a completed orbit nor a path limit
cycle. Establish lower-loop tracking before tightening NPFG.

## Performance: Measured Versus Provisional

| Item | Observation / current setting | Decision |
| --- | --- | --- |
| Mass | 2.000 kg, no payload | Preserve ACF/CG/motor geometry |
| MC climb / descent | About 2 / 1.4 m/s tracked | Preserve |
| CAS / EAS command | Mean 26.46 / 26 m/s | No wrong trim-speed command indicated |
| TAS | Mean 28.61 m/s at density about 1.05 kg/m3 | Normal EAS/TAS difference, not runaway speed |
| FW throttle | Mean about 0.18, not maximum | No maximum-power cruise explanation |
| Trim seeds | FW_THR_TRIM=0.25, FW_PSP_OFF=2 deg | Provisional, not calibrated in steady stabilized flight |
| Stall/min/trim/max | 17 / 21 / 26 / 34 m/s | Actual stall and maximum sustainable speed unmeasured |
| FW climb/sink limits | Existing performance model | No full-power climb/idle descent performed |
| Bank / Hold radius | 16 deg / 1300 m | Freeze for lower-loop comparison, not final agility |
| FW_R_TC | 1.5 s exceeds metadata maximum 1.0 | Explicit configuration debt; do not also raise outer bandwidth |
| Battery | 66.6 to 51.8 Wh over recording | No depletion; fresh battery each test |

At observed TAS a coordinated level 16 deg turn needs roughly 290 m radius;
25 deg would imply about 180 m. These are still-air geometry calculations,
not demonstrated tracking or certified limits. Simulator MC-body alpha near
-86 deg in FW is not a conventional wing stall angle.

PX4's [performance guide](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing)
requires measured trim, speed, climb and idle-descent conditions.
Its [rate/attitude guide](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing)
places lower-loop tuning before guidance. Generic surface-control starting
gains do not calibrate this rotor-only tailsitter.

## R14: Exact Change And Verification

Only flight-profile change: **FW_RR_P 0.14 -> 0.18** (+28.6%). With differential
scale 0.55 unchanged, normalized physical-MC-yaw P changes 0.077 -> 0.099;
these are NOT physical moment gains. I/D/FF, FW_R_TC, NPFG, TECS, speeds, all
MC settings, transition timing, ACF and allocation remain unchanged.
Firmware hash stays identical to R13; no PX4 core change is added.

Independent AI reviews covered FW and MC separately. The FW reviewer initially
suggested +0.01 conservatively, then agreed 0.18 is a more informative bounded
SITL experiment given the weak modal imbalance and available command headroom.
This is not human maintainer approval or a predicted settling-time guarantee.

Follow QUICKSTART.md. Compare the first matched 25 s of FW, then continue to
45 s only if bounded. Desired evidence: lower modal amplitude and late rate
error without worse attitude tracking, new high-frequency motion or clipping.
Roll back to 0.14 if comparable late rate/error RMS worsens over 10% or new
sustained saturation occurs. Do not tighten radius or normalize TC simultaneously.

Allow up to 25 s of visibly DECAYING MC attitude recovery, then Land. Hold may
command translation; zero XY speed is not required to request Land. Stop a
growing departure promptly. Commands, stop criteria and upload retry are in
QUICKSTART.md / COMMANDS.md.

This is not full production acceptance. Once damping is confirmed, validate
straight legs, complete left/right turns, altitude/speed changes, trim and
performance limits under measured conditions before publishing defaults.
Other airframes and the main PR are untouched.
