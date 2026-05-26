# px4xplane Journey Log

This log preserves project decisions, evidence, and next actions across the longer px4xplane recovery effort.

## 2026-05-26

### v3.4.46 qtail22 FW Energy and Back-Transition Recovery

- Reviewed `/home/alireza/qtail22.zip`: TruthCapture recorded `46,913` rows,
  `0` dropped rows, `0` sim-time resets, and about `86 Hz` mean callback rate.
  px4xplane loaded `v3.4.45`, `Config Name: QuadTailsitter`, and body-axis pitot
  `-Z`.
- qtail22 confirmed MC and front transition are no longer the main blocker.
  Front transition completed at about `195.5 s` and the bridge diagnostics did
  not show a sensor-contract regression.
- The failure split is FW energy/guidance and back-transition entry speed:
  fixed-wing TAS commonly sat around `35-40 m/s` while TECS targeted `28 m/s`,
  then back-transition started around `37-55 m/s` and produced a large sink with
  saturated motors.
- Prepared v3.4.46:
  - modestly raises MC climb speed only
  - lowers FW speed and transition energy targets
  - tightens NPFG/roll response from the v3.4.45 guardrail values
  - lengthens/softens back-transition
  - hides the large rendered QuadTailsitter gear while keeping fixed contact
    physics active
- Next action: do a manual FW/back-transition gate only. Do not run full VTOL
  Land missions until manual back-transition is clean.

### v3.4.45 qtail21 MC Acceptance and First-Transition Gate

- Reviewed `/home/alireza/qtail21.zip`: px4xplane loaded `v3.4.44`,
  `Config Name: QuadTailsitter`, body-axis pitot `-Z`, and the packaged
  QuadTailsitter aircraft.
- qtail21 loaded the intended v3.4.44 PX4 defaults. Key values matched:
  `MPC_XY_CRUISE=3.5`, `MPC_XY_VEL_MAX=4.0`, `MPC_ACC_HOR=0.7`,
  `MPC_ACC_HOR_MAX=1.0`, `MPC_JERK_AUTO=0.7`, `MPC_JERK_MAX=1.2`,
  `MPC_TILTMAX_AIR=30`, and `MC_PITCHRATE_D=0.0010`.
- TruthCapture recorded `22,696` rows, `0` dropped rows, `0` sim-time resets,
  about `80 Hz` mean callback rate, and clean pause/resume handling.
- MC acceptance result: trajectory horizontal speed max was `3.5 m/s`, ULog
  actual horizontal speed max was about `4.1 m/s`, motor command peak was about
  `0.345`, and estimator timeout flags stayed `0`.
- Prepared v3.4.45 as a documentation/version/test-gate release only:
  - keep qtail21 MC params and ACF unchanged
  - move next test to one straight forward transition and short FW segment
  - defer FW Orbit/RTL and NPFG/TECS tuning until straight-transition evidence
    is available
  - document that the large landing legs are fixed for contact stability; a
    later visual-only gear slice should not make physics contact pads retract

## 2026-05-25

### v3.4.44 qtail20 MC Overspeed Recovery

- Reviewed `/home/alireza/qtail20.zip`: v3.4.43 removed the stationary baro,
  vertical-velocity, accelerometer-bias, compass, and pause/unpause failures.
- qtail20 remaining issue is MC Go-To tuning. PX4 trajectory speed stayed at or
  below `5 m/s`, but actual horizontal speed reached about `14 m/s` with a
  large pitch transient.
- Found qtail20 did not load the v3.4.43 PX4 qtail softening: ULog still had
  older acceleration/jerk and land-detector defaults. The next test must force
  a clean PX4 parameter load before judging behavior.
- Prepared v3.4.44:
  - keeps the 5 kg ACF unchanged because sizing, thrust, wing area, and contact
    model remain plausible from qtail20 evidence
  - reduces MC auto/manual speed, acceleration, jerk, and tilt limits
  - increases thrust margin for horizontal control
  - adds a small pitch-rate damping increase
- Next action: retest MC only first, then attempt one straight transition only
  if Go-To, RTL/land, and estimator behavior are clean.

### v3.4.43 qtail19 Baro Liveness and Contact Cleanup

- Reviewed `/home/alireza/qtail19.zip`: v3.4.42 removed the earlier stationary
  vertical-velocity, accelerometer-bias, and pause/unpause failures, but
  QuadTailsitter still logged `BARO #0 failed: STALE!` before arming.
- Root cause: the stationary-ground contract froze baro pressure exactly. PX4's
  `DataValidator` treats many identical pressure values as a stuck/stale
  sensor, even when HIL_SENSOR packets are still arriving.
- Prepared v3.4.43:
  - keeps stationary GPS/state velocity, gyro, and acceleration latched/zeroed
  - keeps baro pressure live with tiny simulated sensor noise while stationary
  - updates replay coverage to prove baro liveness without moving GPS altitude
  - stabilizes QuadTailsitter ACF landing contact pads/struts
  - slightly softens QuadTailsitter MC horizontal acceleration and jerk
  - lowers `LNDMC_Z_VEL_MAX` below crawl/speed thresholds
- Next action: repeat the stationary QuadTailsitter and Alia sanity gate, then
  run QuadTailsitter MC-only takeoff, Go-To, RTL, and landing before returning
  to fixed-wing transition work.

### v3.4.40 Ground-Stationary IMU Guard

- Paused QuadTailsitter tuning after a stationary-connection regression:
  PX4 reported vertical velocity instability immediately after SITL connect.
- Reviewed `/home/alireza/20260525-044651Z`: TruthCapture showed the aircraft
  remained on the ground, unpaused, with zero throttle, zero prop rotation, and
  zero prop thrust. X-Plane still reported large ground-contact acceleration
  spikes, including `g_nrml` up to about `6.29 g` and `local_ay` up to about
  `51.9 m/s^2`.
- Root cause: px4xplane forwarded X-Plane gear/contact g-load spikes directly
  into HIL_SENSOR acceleration while the aircraft was stationary.
- Prepared v3.4.40:
  - default-on `ground_stationary_accel_guard_enabled`
  - stationary-on-ground detection using X-Plane on-ground, AGL, vertical speed,
    and horizontal speed
  - replacement acceleration of stable FRD gravity while the guard is active
  - config path logging throttled to path changes only
- Next action: install v3.4.40 and do a stationary connect test before resuming
  QuadTailsitter MC/FW tuning.

### v3.4.41 Ground-Stationary Kinematics Guard

- Follow-up feedback: v3.4.40 reduced but did not eliminate the stationary
  vertical-velocity warning. Alia showed smaller vertical-speed spikes, while
  QuadTailsitter was worse.
- Diagnosis: v3.4.40 guarded acceleration only and used vertical speed as part
  of the guard gate. That means a contact-model vertical-speed spike disabled
  the guard exactly when needed, and GPS/HIL_STATE velocity still carried the
  spike into PX4.
- Prepared v3.4.41:
  - added `ground_stationary_kinematics_guard_enabled`
  - zeroes local NED velocity for GPS and HIL_STATE while stationary on ground
  - uses the same guarded velocity for GPS course-over-ground
  - applies the same guarded velocity to body-axis pitot projection
  - removes vertical-speed from the stationary-ground acceleration guard gate
- Inspected QuadTailsitter ACF landing gear. The aircraft has long angled gear,
  large strut compression values, and very small tire/contact dimensions, so it
  likely amplifies X-Plane contact jitter. Deferred ACF gear changes until the
  reusable bridge contract is re-tested.

### v3.4.42 Stationary-Ground Sensor Contract

- Reviewed `/home/alireza/qtail17.zip` after v3.4.41 still produced stationary
  estimator warnings. The remaining evidence showed a mixed stationary sensor
  story: acceleration and GPS velocity were guarded, but GPS/baro altitude,
  gyro rates, and some attitude/contact cases were still raw.
- Root-cause split:
  - bridge bug: stationary ground contact must be one coherent zero-motion
    sensor contract, not independent per-topic guards
  - aircraft amplifier: QuadTailsitter gear/contact geometry likely increases
    X-Plane jitter, but should be tuned only after the bridge sanity gate is
    clean
  - future timing item: EKF time slip still needs dedicated replay/low-FPS work
    if warnings remain after the sensor contract
- Prepared v3.4.42:
  - latched stationary ground state with hysteresis
  - motor-command release gate to avoid hiding real takeoff/thrust dynamics
  - attitude-consistent gravity instead of level-only gravity
  - zero gyro rates, GPS/state velocity, and latched GPS/state/baro altitude
  - GPS/baro altitude noise suppressed only during stationary ground contract
  - replay tool and unit test coverage for the contract
- Next action: perform a stationary connect/pause/arm-disarm sanity test on
  QuadTailsitter and Alia before any further QuadTailsitter aero or gear tuning.

## 2026-05-23

### v3.4.35 QuadTailsitter qtail12 FW Orbit and Pitot Density

- Reviewed `/home/alireza/qtail12.zip`: the zip passes integrity checks and
  TruthCapture recorded `61,908` frames with zero dropped rows, about `80 Hz`
  mean callback rate, and a `50 ms` worst frame period.
- Confirmed the run used `SYS_AUTOSTART=5021`,
  `airspeedSource=body_axis`, `pitotAxisBody=-Z`, `FW_USE_AIRSPD=1`, and
  `SYS_HAS_NUM_ASPD=1`.
- Confirmed PX4 used the modeled pitot, not open-loop transition:
  `airspeed_validated.airspeed_source=1` throughout, and fixed-wing transition
  completed when calibrated airspeed reached about `20 m/s`.
- Found the FW orbit/RTL problem was mainly guidance saturation at speed:
  qtail12 flew about `32 m/s` in FW hold and `37 m/s` during RTL, while
  lateral acceleration repeatedly hit the `35 deg` roll-limit equivalent.
  The allocator achieved main torque setpoints; the log does not justify motor
  cant as the next immediate fix.
- Corrected body-axis pitot dynamic pressure to use X-Plane local
  pressure/temperature density. X-Plane IAS mode remains sea-level-equivalent,
  matching the indicated/equivalent airspeed convention.
- Prepared v3.4.35 to lower qtailsitter FW trim/throttle, transition at
  `18 m/s` calibrated airspeed, widen FW/RTL loiter to `500 m`, and soften
  NPFG (`PERIOD=22`, `DAMPING=0.85`, `ROLL_TC=1.2`).

### v3.4.34 QuadTailsitter qtail11 Body-Axis Pitot and Path Recovery

- Reviewed `/home/alireza/qtail11.zip`: valid ULog, valid TruthCapture,
  `45,956` truth frames, no dropped rows, about `77 Hz` mean callback rate.
- Confirmed qtail11 did not use the modeled pitot for PX4 control. The run
  still had `airspeedSource=xplane_indicated`, `FW_USE_AIRSPD=0`,
  `SYS_HAS_NUM_ASPD=0`, and `VT_ARSP_TRANS=0`.
- Validated the physical pitot-axis candidate from truth data: `-Z` body-axis
  projection is positive in FW and close to X-Plane true airspeed; X-Plane's
  conventional IAS dataref is negative in this tailsitter attitude.
- Found FW path following was not mainly a missing-bank problem. In tailsitter
  FW coordinates, PX4 requested up to `+/-35 deg` roll and actual transformed
  roll followed reasonably. The main issue was flying about `42 m/s` against a
  `250 m` loiter radius, which is near the physical limit and uncomfortable if
  the target bank is only `12-15 deg`.
- Deferred aircraft mass and motor-cant changes. Current ACF mass is `5 lb`
  (`2.27 kg`), matching Quantix-class public specs, and the log does not prove
  torque authority as the FW path root cause. Motor cant would require matching
  PX4 `CA_ROTOR*_AX/AY/AZ` geometry.
- Prepared v3.4.34 to enable body-axis pitot feedback, use an airspeed-gated
  transition at `20 m/s`, target `24 m/s` FW trim speed, widen FW loiter to
  `300 m`, soften back-transition, and remove the land-detector startup
  correction.

## 2026-05-22

### v3.4.33 QuadTailsitter qtail10 FW Energy Recovery

- Reviewed `/home/alireza/qtail10.zip`: valid ULog, valid TruthCapture,
  `45,719` truth frames, no dropped rows, about `80 Hz` mean callback rate.
- Confirmed the first forward-transition failure was expected because it was
  attempted around `26 m` AGL while `VT_FW_MIN_ALT=40`.
- Confirmed the second transition reached FW state, but the FW segment oversped
  around `40-60 m/s` while PX4's airspeedless controller used a synthetic
  `18 m/s` airspeed reference.
- Found the `120 m` FW loiter radius was not compatible with qtail10 speed:
  at `50 m/s`, even a `45 deg` bank needs about `255 m` radius.
- Kept the airspeedless control policy for this test, but added reusable
  `airspeedSource` and `pitotAxisBody` config/schema support for later physical
  pitot-axis validation.
- Prepared v3.4.33 to reduce FW throttle/energy, use a `250 m` FW loiter
  radius, smooth MC Go-To acceleration, and make MC landing less slow while
  keeping a final crawl.

### v3.4.32 QuadTailsitter qtail9 Transition Airspeed Recovery

- Reviewed `/home/alireza/qtail9.zip`: valid ULog, valid TruthCapture, about
  `650.8 s` of PX4 data and `54,342` truth frames at about `83 Hz`.
- Found qtail9 did not use clean v3.4.31 defaults. `SYS_AUTOCONFIG=0` and
  persistent/manual values were active, including `MPC_TILTMAX_AIR=45`,
  `MPC_VEL_MANUAL=5`, `MC_YAWRATE_P=0.20`, and `MC_YAWRATE_D=0.04`.
- Confirmed Go-To speed changes were mostly PX4 `DO_REPOSITION` point-hold
  behavior: each clicked target accelerates toward the point, decelerates near
  it, then the next clicked target accelerates again.
- Found six forward-transition attempts. Each entered transition-to-FW and
  returned to MC; none reached fixed-wing state. The main blocker was
  `FW_USE_AIRSPD=1` plus `VT_ARSP_TRANS=15` while validated airspeed stayed
  near zero.
- Truth capture showed X-Plane IAS ranged from about `-134 kt` to `+8 kt`
  while true airspeed reached about `76 m/s`. px4xplane had been clamping
  negative IAS to zero before calculating differential pressure.
- Implemented signed differential pressure in the bridge and replay tool,
  following PX4's signed `differential_pressure` topic semantics. This keeps
  high-AoA/reverse-flow evidence visible without faking airspeed from TAS.
- Prepared a first-transition QuadTailsitter candidate that disables transition
  airspeed gating for this airframe, slows pitch-over, raises transition safety
  margins, and promotes moderated live MC yaw/tilt authority.

### v3.4.31 QuadTailsitter qtail9 Go-To Smoothness Polish

- Reviewed `/home/alireza/05_30_57.ulg`. The log was valid: about `366.9 s`,
  full topic set, successful takeoff, Go-To, Orbit, RTL, landing, disarm, and
  no crash.
- Confirmed the run did not use clean v3.4.30 defaults. Persistent/manual
  values were active at boot, including `MPC_XY_VEL_MAX=5`,
  `MPC_VEL_MANUAL=5`, `MPC_TILTMAX_AIR=45`, `MPC_XY_P=0.15`,
  `MPC_Z_P=1.0`, `MC_YAW_P=0.8`, `MC_YAWRATE_P=0.2`,
  `MC_YAWRATE_D=0.04`, and roll/pitch rate `K=1.0`.
- Found the user's Go-To observation in the data. The main visible
  brake/continue behavior came from PX4 point-reposition behavior: the
  trajectory setpoint deliberately slowed near the Go-To target, then the next
  Go-To accelerated again. This is expected for `DO_REPOSITION`, not a blended
  waypoint path. A secondary tracking issue remains: during parts of the Go-To,
  actual speed fell below demanded speed while motors had headroom and pitch
  lagged the setpoint by about `15-19 deg`.
- Found the Orbit segment was much healthier than qtail8: yaw/attitude errors
  were small and no thrust saturation sequence appeared.
- Prepared v3.4.31 as a parameter-only polish: keep the successful speed
  direction, strengthen roll/pitch/yaw authority moderately, document Go-To as
  point-reposition behavior, and avoid bridge, aircraft-geometry,
  canted-motor, FW, or transition changes until MC Position/Go-To/Orbit is
  clean at `4-5 m/s`.

### v3.4.30 QuadTailsitter qtail8 Agile MC Recovery

- Reviewed `/home/alireza/qtail8.zip`. The package was partly corrupt:
  `04_17_10.ulg` had a bad CRC and TruthCapture `frames.csv` failed inflate,
  so tuning conclusions use the valid `04_31_13.ulg` and TruthCapture metadata.
- Confirmed the usable log started airborne and did not represent clean v3.4.29
  defaults. It already had `MPC_XY_VEL_MAX=5`, `MPC_VEL_MANUAL=5`,
  `MPC_TILTMAX_AIR=45`, `MC_YAWRATE_K=1.2`, and yaw D active.
- Found the live experiments did contain useful information: with
  `MPC_XY_CRUISE=3`, the vehicle reached about `2.8 m/s` p95 and `3.4 m/s`
  max in Go-To while maintaining altitude within about `0.6 m` p95.
- Found the crash sequence: a `30 m`, center-facing Orbit command at low
  altitude caused rapid yaw motion, vertical sink, motor saturation,
  thrust-not-achieved, ground contact, and then roll attitude failure. Estimator
  health was clean until after impact.
- Prepared v3.4.30 as an agile-but-guarded MC slice: faster than v3.4.29,
  capped at `25 deg` tilt, moderate yaw gain increase, slower auto yaw setpoint
  motion, and moderate altitude-hold gain changes.
- Deferred connection-code and aircraft-geometry changes. The usable log did
  not prove a bridge sensor failure or a new simulator-TCP regression.

### v3.4.29 QuadTailsitter qtail7 Orbit Damping

- Reviewed `/home/alireza/qtail7.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.28`, and completed takeoff, Go-To, Orbit, RTL, landing, and
  disarm without a crash.
- TruthCapture was healthy: `28,643` frames over about `352.7 s`, about
  `81.2 Hz` mean callback rate, zero dropped rows, no sim-time resets, and max
  frame period about `50 ms`.
- Found no estimator-rooted failure: in-flight innovation ratios stayed low and
  estimator-vs-truth attitude errors were small enough for this phase.
- Found the remaining Go-To and Orbit problem is yaw/attitude tracking lag:
  Go-To yaw p95 was about `22.6 deg`, Orbit yaw p95 about `38.6 deg`, and
  Orbit roll/pitch p95 about `18-20 deg` during early capture.
- Confirmed PX4 control allocation reported torque achieved and motors had
  headroom, so this slice keeps the aircraft geometry unchanged and defers
  canted-motor work to a later controlled sweep.
- Prepared v3.4.29 as a parameter-only orbit-damping slice: stronger yaw-rate
  tracking, slightly stronger roll/pitch rate P, lower yaw `KM` magnitude to
  request more yaw motor differential, and softer low-speed XY trajectory
  limits.
- Rechecked the connection concern. Current code still has the v3.4.20
  accepted-socket nonblocking regression reverted; a QGC parameter progress
  hang should be triaged on the GCS-PX4 MAVLink/cache path before changing
  px4xplane simulator-TCP code again.

## 2026-05-21

### v3.4.28 QuadTailsitter qtail6 Hover Yaw Polish

- Reviewed `/home/alireza/qtail6.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.27`, and flew the first QuadTailsitter takeoff, two Go-To
  commands, landing, and disarm without a crash.
- TruthCapture was healthy: `16,866` frames, about `79.8 Hz` mean callback,
  zero dropped rows, and no sim-time resets.
- Confirmed qtail6 fixed the previous allocator/control-authority failure:
  no sustained unallocated torque, no sustained motor saturation, and clean
  estimator test ratios.
- Found the near-ground takeoff pause was commanded by `MPC_TKO_SPEED=0.6` and
  `MPC_TKO_RAMP_T=2.5`; the local setpoint stayed near the ground for the first
  seconds after takeoff before climbing to QGC's commanded altitude.
- Found the remaining Go-To wobble is mostly yaw lag. First-leg yaw p95 error
  was about `30.7 deg`, while roll/pitch errors stayed in the low single-digit
  degree range and position tracking was usable for a recovery tune.
- Prepared v3.4.28 as a param-only polish: faster but still conservative
  takeoff/climb, modest yaw authority increase, and no ACF/canted-motor change.
- Documented motor cant as a valid next physical-model slice if qtail7 still
  shows yaw lag with clean motor headroom.

### v3.4.27 QuadTailsitter qtail5 Hover Authority Recovery

- Reviewed `/home/alireza/qtail5.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.26`, and had the intended `CT=6.5` yaw-allocation fix active.
- TruthCapture was healthy: `10,263` frames, about `77 Hz` effective callback,
  zero dropped rows, and no sim-time resets.
- Confirmed qtail4's yaw allocation fault was fixed: yaw motor differential was
  present and `control_allocator_status` no longer reported persistent
  unallocated yaw torque.
- Found the new dominant failure: `CT=6.5` overestimated the current X-Plane
  aircraft's pitch/roll effectiveness. PX4 reported torque achieved while
  motor differentials stayed too small and the aircraft slowly entered
  high-speed tilted flight before recovery could catch up.
- Confirmed this is not a 6 kg aircraft yet. The current ACF is `2.27 kg`, and
  TruthCapture showed only about `43-47 N` total thrust near full throttle,
  which is not enough for the requested future `6-6.5 kg` physical model.
- Changed `CA_ROTOR*_CT` to `2.0`, increased pitch/roll rate authority, reduced
  yaw demand for the lower CT, softened horizontal/tilt recovery commands, and
  raised nominal hover thrust to reduce the near-ground pause.

### v3.4.26 QuadTailsitter qtail4 Yaw Allocation Recovery

- Reviewed `/home/alireza/qtail4.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.25`, and crashed after yaw drift grew into pitch/roll
  attitude failure.
- TruthCapture was healthy: `11,609` frames, about `77 Hz` effective callback,
  zero dropped rows, and no sim-time resets. The current ACF mass was
  `2.27 kg`, not the proposed `6-6.5 kg` physical redesign.
- Found the specific allocator fault: the active airframe used
  `CA_ROTOR*_CT=1.0` and `CA_ROTOR*_KM=+/-0.05`. PX4's allocator zeros rows
  with no effectiveness entry above `0.05`, so the yaw row landed exactly on
  the cutoff and could be removed.
- Confirmed the signature in ULog: yaw torque was requested, yaw torque was
  reported unallocated, and `actuator_motors` kept the yaw differential near
  zero even while yaw error grew.
- Restored `CA_ROTOR*_CT=6.5`, reduced yaw demand, set hover thrust to `0.25`,
  shortened takeoff ramp to `3 s`, and made the next validation a `1.5 m`
  hover-only test.
- Deferred the requested 6 kg / 10 inch prop / canted-motor ACF redesign to a
  separate slice after stable hover is proven, so this package isolates the
  allocator fix.

## 2026-05-20

### v3.4.25 QuadTailsitter qtail3 Yaw Authority Recovery

- Reviewed `/home/alireza/qtail3.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.24`, and no longer had the violent first-lift roll/pitch
  oscillation.
- TruthCapture was healthy: `9,261` frames, about `84.0 Hz` mean callback, zero
  dropped rows, and no sim-time resets. The aircraft mass stayed at `2.27 kg`,
  consistent with the Quantix-class `5 lb / 2.3 kg` reference size.
- Found the remaining slow divergence was yaw authority, not mass/CG/airfoil or
  FPS. Between about `30-53 s`, motors stayed around `0.21-0.31`, yaw error
  grew to about `70 deg`, PX4 commanded about `28 deg/s` yaw correction, and
  control allocation reported yaw torque as unallocated.
- Restored `MC_AIRMODE=2` while keeping the reduced qtail3 roll/pitch gains,
  moderately raised yaw response, set `MIS_TAKEOFF_ALT=3.0`, and aligned
  `LNDMC_Z_VEL_MAX=0.25` with the actual PX4 parameter clamp.
- Checked the connection wait: X-Plane listened on port `4560` for about `25 s`
  before PX4 connected, then PX4 became ready quickly. This log does not show a
  broken telemetry/stream path.

### v3.4.24 QuadTailsitter qtail2 Hover Dynamics Recovery

- Reviewed `/home/alireza/qtail2.zip`. PX4 used `SYS_AUTOSTART=5021`, loaded
  px4xplane `v3.4.23`, and had the corrected qtail1 rotor geometry active.
- TruthCapture was healthy: `9,138` frames, about `75.2 Hz` mean callback, zero
  dropped rows, and no sim-time resets.
- Found that qtail2 was not a simple FPS, sensor, or motor-order failure. In
  the first `10 s` after takeoff, motor outputs were not saturated, but roll
  already reached about `-35/+19 deg` and p99 roll rate about `136 deg/s`.
- Found hidden aircraft-side control authority in the X-Plane model:
  Plane Maker Artificial Stability had P/Q/R application enabled for all four
  engines. v3.4.24 source-controls the QuadTailsitter aircraft and disables
  those engine-stability paths for external-autopilot use.
- Reduced the QuadTailsitter PX4 hover tune aggressively for a hover-only
  recovery pass: lower attitude/rate gains, lower rate limits, lower yaw demand,
  disabled `MC_AIRMODE`, slower takeoff, lower tilt/acceleration/jerk limits.
- Fixed the startup land-detector correction by setting
  `LNDMC_Z_VEL_MAX=0.20`, below the landing crawl speed.

### v3.4.23 QuadTailsitter qtail1 Hover Recovery

- Reviewed `/home/alireza/qtail1.zip`. PX4 used `SYS_AUTOSTART=5021`, armed at
  `28.15 s`, detected takeoff at `29.37 s`, and reported roll attitude failure
  at `33.22 s`.
- TruthCapture was not the cause: `5,357` frames, about `82.3 Hz` mean callback,
  no dropped rows, and no sim-time resets.
- Found the first-order fault in the QuadTailsitter control-allocation geometry.
  The previous file copied X-Plane ACF motor coordinates directly; X-Plane
  aircraft axes are X=right, Y=up, Z=tail, while PX4 allocation needs body
  X=forward and body Y=right in meters.
- Corrected the rotor positions to the converted motor geometry and reduced
  multicopter rate/attitude aggressiveness for a hover-only recovery test.
- Kept the ACF flight physics unchanged for this slice. The log supports fixing
  allocator/tuning first; changing motor/wing/mass physics at the same time
  would hide whether the root cause was actually fixed.
- Cleaned user-visible naming to `QuadTailsitter`. The PX4 build target and
  airframe file remain `xplane_qtailsitter` / `5021_xplane_qtailsitter`.

### v3.4.22 evtol7 Milestone And QuadTailsitter Prep

- Reviewed `/home/alireza/evtol7.zip`. `08_44_10.ulg` is Alia
  (`SYS_AUTOSTART=5020`); `08_51_36.ulg` is Ehang (`SYS_AUTOSTART=5010`).
- Accepted Alia as the milestone baseline. The only live back-transition change
  in the accepted Alia run was `VT_B_TRANS_RAMP=15.0`, so v3.4.22 adopts that
  ramp while keeping the accepted front-transition, TECS, NPFG, bank, and
  no-brake defaults unchanged.
- Found the remaining late disarm is not a flight-control issue. Both Alia and
  Ehang briefly toggled `landed=false` after touchdown, then disarmed 2 s after
  the final stable `landed=true`. v3.4.22 sets `COM_DISARM_LAND=1.5` for the
  two SITL eVTOL airframes.
- Confirmed Ehang's successful live tuning sequence ended at
  `MC_ROLLRATE_K=3.0`, `MC_PITCHRATE_K=2.05`, `MC_ROLL_P=1.0`, and
  `MC_PITCH_P=1.0`. These become the final Ehang defaults for the milestone.
- The first Alia connection timeout in X-Plane `Log.txt` was a listen/retry
  timing issue, not a flight-stack fault. The plugin is left on the recovered
  v3.4.21 stream path, but the user-visible wait window is extended to 60 s.
- Audited the uploaded QuadTailsitter source `.acf`: it is a four-motor, no-control-surface
  tailsitter with about `5-6 lb` mass and motors near `+/-1.4 ft` lateral
  and `+/-0.72 ft` longitudinal offsets. The old `5021_xplane_qtailsitter`
  comments overclaimed readiness and used faster-than-official first-test
  speeds.
- Rebuilt the QuadTailsitter parameter file from PX4's official
  quadtailsitter SITL baseline, with px4xplane estimator settings and a
  conservative first-test envelope (`FW_AIRSPD_TRIM=18`, `VT_ARSP_TRANS=15`,
  `NAV_LOITER_RAD=120`).

### v3.4.21 SITL Connection Recovery And evtol6 Follow-Up

- Reviewed `/home/alireza/evtol6.zip`. The flight evidence in the archive used
  v3.4.19, not v3.4.20, so v3.4.20's QGC/telemetry failure was traced from the
  connection code delta and the user's terminal observation.
- Identified the v3.4.20 regression: after accepting PX4's simulator TCP
  connection, the plugin made the connected stream non-blocking and could drop
  a complete MAVLink packet when `send()` would block. That is unsafe for the
  simulator MAVLink stream even if commands sometimes still pass.
- Restored the connected stream behavior to the v3.4.19 baseline while keeping
  the cancellable waiting menu/HUD flow.
- Confirmed Alia's v3.4.19 fixed-wing path/altitude behavior remains the
  protected baseline. No TECS, NPFG, fixed-wing bank, or front-transition
  retune was made.
- Found Alia's airspeed warning is a startup health-check race: the virtual
  pitot is valid once the simulator stream is warm, but `SYS_HAS_NUM_ASPD=1`
  makes PX4 report a physical-airspeed prearm failure before that. v3.4.21 keeps
  airspeed control/checking active but sets `SYS_HAS_NUM_ASPD=0` for the SITL
  airframe.
- Found delayed disarm in Alia/Ehang was land-detector bouncing. `landed=true`
  dropped false when estimated vertical speed briefly reached about
  `0.50-0.54 m/s`; raised final crawl and land-detector velocity headroom.
- Adopted the user-tested Alia `VT_B_DEC_MSS=1.5`; the ULog did not show the
  claimed `VT_B_TRANS_DUR=10`, so duration stays at `35.0`.
- Ehang still had Alia's old takeoff gate problem (`MPC_XY_ERR_MAX=2`) and
  under-responsive Orbit pitch/roll behavior. v3.4.21 raises the gate to `10`,
  moderately raises attitude response to `0.45`, and reduces high velocity D
  damping to `0.8`.

### v3.4.20 SITL Connection Reliability

- Reviewed the px4xplane connection lifecycle after user feedback that SITL can
  connect immediately in some runs and after roughly `5-10 s` in others.
- The current architecture is correct for PX4's simulator TCP model: px4xplane
  listens on port `4560`, PX4 `simulator_mavlink` connects as the client, and
  the plugin polls non-blocking `accept()` once per X-Plane flight loop.
- The observed few-second wait is expected when PX4 was already running before
  the plugin began listening, because PX4's client side may be between retry
  attempts. The plugin cannot force that retry earlier, but it can make the
  wait/cancel/error states clear and safe.
- Fixed the confusing waiting-state menu. While the server socket is listening,
  the menu now says `Cancel SITL Connection Wait` and closes the listener if
  selected.
- Hardened socket handling: accepted PX4 sockets are non-blocking,
  `TCP_NODELAY` is enabled, stale send/receive/select failures trigger clean
  disconnect, and plugin unload also closes a waiting listener.
- Removed `SO_REUSEPORT`; quick reconnect still uses `SO_REUSEADDR`, but a
  second listener on port `4560` should fail visibly instead of creating
  ambiguous ownership.
- No Alia or Ehang PX4 parameter changes were made in this slice. v3.4.19
  tuning remains the flight-test baseline.

## 2026-05-19

### v3.4.19 Alia Takeoff Gate And Ehang Roll Recovery

- Reviewed `/home/alireza/evtol5.zip`.
- Alia was a valid `SYS_AUTOSTART=5020` run and had no in-flight PX4 warnings.
  FW loiter/RTL altitude and lateral behavior matched the intended no-brake
  baseline, so fixed-wing and transition parameters remain frozen.
- The only Alia issue was takeoff acceptance delay. The ULog showed
  `MPC_XY_ERR_MAX=2` stopping vertical setpoint integration while horizontal
  error stayed above the default threshold; changed only this MC takeoff gate to
  the PX4 maximum `10`.
- Confirmed Alia `autoPropBrakes` was disabled in X-Plane Log.txt, with
  `Motor brakes configured for motors: 00000000`. TruthCapture showed the
  pusher active in FW and lift motors near idle with low residual rotation.
- Ehang `evtol5` was a valid `SYS_AUTOSTART=5010` run and failed from control
  tuning, not mapping. Actual roll exceeded `70 deg` while setpoint was capped
  around `25 deg`, motors saturated, and PX4 reported repeated roll attitude
  failures.
- Rolled Ehang back to the last non-oscillatory MC attitude/position baseline
  instead of adding another speculative gain increase.
- Added active airframe display to the waiting HUD and menu-friendly display
  names, while keeping `config.ini` section names and PX4 make targets unchanged.

### v3.4.18 Alia Brake Recovery And Ehang Orbit Damping

- Reviewed `/home/alireza/evtol4.zip`.
- Alia pusher thrust was present: PX4 commanded output 8 at full throttle and
  TruthCapture showed X-Plane engine 4 producing about `4 kN` thrust in FW.
- The Alia crash was not an airspeed loss. `airspeed_validated` stayed live and
  matched raw airspeed at the FW switch. The aircraft initially recovered the
  same first-FW sink shape as v3.4.13, then the lift-prop brake window removed
  the remaining margin and the aircraft entered a fatal sink.
- Restored Alia to the accepted no-brake baseline by disabling default
  `autoPropBrakes`, returning `VT_ARSP_TRANS=46.0`, and returning
  `FW_PSP_OFF=3.0`. Kept `ASPD_DO_CHECKS=1`.
- Ehang Orbit was not a missing roll-channel or motor-mapping failure. The
  ULog showed radial velocity setpoints dominating the slow Orbit command and
  pitch setpoint repeatedly reaching `+-30 deg` while actual pitch lagged.
- Corrected Ehang's under-responsive MC attitude gains and reduced the high
  horizontal velocity D gain so the next test judges a damped Orbit response.

### v3.4.17 Alia Airspeed And Reconnect Follow-Up

- Reviewed `/home/alireza/evtol3.zip`.
- Alia raw `differential_pressure` and raw `airspeed` were present; raw IAS
  reached about `58 m/s`. The failure was that `airspeed_validated` stopped
  publishing at `14.35 s`, so PX4/QGC kept showing stale near-zero validated
  airspeed and Standard VTOL never completed the FW switch.
- Updated Alia to `ASPD_DO_CHECKS=1` for the SITL virtual pitot. This keeps
  missing-data detection but avoids validator checks that are not useful while
  a VTOL is intentionally at near-zero airspeed in multicopter mode.
- Patched the PX4 fork `airspeed_selector` to avoid indexing the sensor
  validator array when the selected source is disabled, ground-minus-wind, or
  synthetic.
- Fixed the px4xplane first-reconnect disconnect after aircraft changes by
  resetting `lastFlightTime` before opening a new SITL server socket.
- Ehang `evtol3` used a `146.47 m` orbit radius with the aircraft starting
  about `518 m` from the center. It spent most of the run capturing the orbit;
  at `5 m/s` and `146 m` radius, expected lateral tilt is only about `1 deg`.

### v3.4.16 Alia/Ehang Follow-Up

- Reviewed `/home/alireza/evtol2.zip`.
- Ehang 184 completed the validation flow cleanly. The initial zigzag after
  QGC Orbit was orbit-capture behavior: the vehicle started outside the
  commanded circle and PX4 first flew to the closest point on that circle.
- Ehang post-capture orbit hunting was reduced by lowering horizontal
  cruise/max/manual speeds and horizontal acceleration limits.
- Alia transitioned to FW then sank until quadchute. ULog and TruthCapture
  showed TECS asking for full throttle/nose-up while actual pitch and altitude
  recovery stayed poor. The hard-lock prop-brake window correlated with the
  failure to recover.
- The prop-brake behavior was made explicitly configurable: `feather` for
  normal retest, `hard_lock` for the old invasive low-level refs, and
  `prop_separate` for controlled X-Plane failure-based A/B testing.
- Alia transition margin was increased with `VT_ARSP_TRANS=50.0` and
  `FW_PSP_OFF=4.0`. Do not claim this as real ALIA flight-quality tuning; it is
  the next X-Plane model validation step backed by the latest logs.

## 2026-04-28

### Context

- Main target remains a production-grade PX4 + X-Plane SITL bridge.
- Current bridge can fly in some slow/hover cases, so working behavior must be preserved until evidence proves a replacement path.
- Existing code comments, docs, and previous AI-generated explanations are treated as hypotheses, not truth.
- X-Plane dataref semantics must be measured on real X-Plane installations because docs and version behavior can be incomplete or stale.

### Public Communication

- Posted short follow-up comments to open px4xplane issues: `#1`, `#2`, `#3`, `#5`, `#6`, `#7`, `#8`, `#9`, `#10`, `#12`, `#13`, `#15`, `#17`.
- Posted a status update to PX4 PR `PX4/PX4-Autopilot#22493`.
- Message tone: work is resuming, logs are requested, no final fix is claimed yet.

### Evidence Added

- Uploaded ULog: `/home/alireza/log_155_2026-1-27-16-58-38.ulg`.
- ULog identity: `13200_generic_vtol_tailsitter`, `is_vtol=true`, `is_vtol_tailsitter=true`.
- Caveat: the ULog is not a PX4 HIL/SITL log. It appears to be NuttX hardware (`PX4_FMU_V5`) with `vehicle_status.hil_state=0` and `MAV_USEHILGPS=0`.
- ULog still helps with PX4 units and estimator symptoms:
  - PX4 sensor accel units are `m/s^2`.
  - Typical processed accel is near gravity magnitude.
  - The log shows estimator/mag stress and failsafes, so it is not a clean validation run.
- MAVLink/PX4 contract evidence:
  - `HIL_SENSOR` accel/gyro/mag are consumed directly by PX4 simulator code.
  - `HIL_GPS.id` is used to select/create GPS instances and must be stable.
  - `HIL_STATE_QUATERNION` acceleration fields are in milli-g, not `m/s^2`; current px4xplane handling is strongly suspected to be unit-inconsistent for that message.

### New Tooling Repo

- Created public repository: https://github.com/alireza787b/xplane-truth-capture
- Purpose: read-only X-Plane dataref/timing capture plugin, independent of PX4 and independent of the existing px4xplane bridge.
- Current release: https://github.com/alireza787b/xplane-truth-capture/releases/tag/v0.1.2
- Release assets:
  - `XPlaneTruthCapture-Windows.zip`
  - `XPlaneTruthCapture-macOS.zip`
  - `XPlaneTruthCapture-Linux.zip`
  - SHA256 files for each platform
- Build status:
  - Main build workflow passed on Windows, macOS, and Linux.
  - Release workflow passed for `v0.1.2`.

### XPlaneTruthCapture Decisions

- X-Plane bundle structure follows standard plugin convention:
  - `XPlaneTruthCapture/64/win.xpl`
  - `XPlaneTruthCapture/64/mac.xpl`
  - `XPlaneTruthCapture/64/lin.xpl`
- The plugin folder has no extension; the platform binary has `.xpl`.
- The plugin is read-only by default.
- It writes run bundles under `X-Plane/Output/XPlaneTruthCapture/<run_id>/`.
- Default datarefs are now editable in `config/default_datarefs.txt`, using:
  - `dataref_path|group|required`
- Extra user datarefs can be added in `config/datarefs.txt`.
- Runtime config lives in `config/capture_config.ini`.

### First XPlaneTruthCapture Run Received

- Uploaded archive: `/home/alireza/20260428-123608Z.rar`.
- Extracted analysis folder: `/tmp/xplane11-capture-20260428/20260428-123608Z/`.
- Capture identity:
  - XPlaneTruthCapture `v0.1.1`
  - X-Plane 11.55r2, XPLM 303
  - Windows
  - Laminar Aerolite 103 fixed-wing
  - capture mode `every_frame`
- Bundle quality:
  - `9320` rows
  - `0` rows dropped
  - about `388.4` seconds of flight time
  - no maneuver markers except start/stop
- Timing evidence:
  - mean frame period about `0.0417 s`
  - median FPS about `24.1`
  - observed FPS range about `19.9` to `29.9`
  - confirms low-FPS support must be a core bridge design requirement.
- Dataref evidence:
  - `102/111` requested datarefs existed.
  - XP12 weather refs were absent in XP11, as expected.
  - XP11 legacy weather refs worked.
  - byte-array aircraft metadata needs proper `XPLMGetDatab` decoding in the logger.
- Sensor/contract evidence:
  - local position finite differences match `local_vx/local_vy/local_vz`, supporting local velocity as the GPS velocity source.
  - derived magnetic track from local velocity matches X-Plane `ground_track_mag` during movement.
  - `g_nrml` is about `+1 g` at rest, supporting negative Z conversion into PX4 FRD, but X/Y accel signs remain unresolved without controlled maneuvers.
  - `indicated_airspeed` raw equals cockpit IAS in knots in this XP11 run.
  - `true_airspeed` raw behaves like m/s in this XP11 run.
  - `psi + magnetic_variation` matches X-Plane magnetic heading within about `0.2 deg`.
- Current-code risks strengthened:
  - `HIL_GPS` is not zero-initialized and appears to lack a stable `id`.
  - `HIL_GPS.cog` currently uses magnetic ground track while MAVLink/PX4 GPS course should be treated as true/earth course over ground unless proven otherwise.
  - GPS timing debug likely reports misleading dt/rate because `lastGPSTime` is overwritten before delta calculation.
  - `HIL_STATE_QUATERNION.ind_airspeed` uses raw knots as cm/s.
  - `HIL_STATE_QUATERNION` accel fields use m/s^2 where MAVLink expects mG.
  - `HIL_STATE_QUATERNION` forwards X-Plane quaternion directly without documented EUS/NED and aircraft/PX4 body-frame conversion.
  - current `HIL_SENSOR` send path is capped by X-Plane flight-loop FPS even when config requests 200 Hz.
  - primary accel X/Y sign still needs controlled proof; do not patch by guessing.
- Next report: `report_v5.md`.
- Next tooling action: update XPlaneTruthCapture to `v0.1.2` with data-byte decoding, config copy, first-row handling, cycle clarity, analyzer output, and better marker/test instructions.

### XPlaneTruthCapture v0.1.2 Released

- Release: https://github.com/alireza787b/xplane-truth-capture/releases/tag/v0.1.2
- Commit: `b86712c Release truth capture v0.1.2`
- GitHub Actions release workflow passed on:
  - Linux
  - Windows
  - macOS
- Assets:
  - `XPlaneTruthCapture-Windows.zip`
  - `XPlaneTruthCapture-macOS.zip`
  - `XPlaneTruthCapture-Linux.zip`
  - SHA256 files
- v0.1.2 additions:
  - self-contained `viewer.html`
  - offline `tools/analyze_capture.py`
  - `analysis_summary.json`, `dataref_stats.csv`, `derived.csv`, `issues.jsonl` outputs from analyzer
  - data-byte decoding through `XPLMGetDatab`
  - copied runtime config in each run folder
  - `unit_hint` in `datarefs.csv`
  - `first_frame` and `xplm_cycle_number` in `frames.csv`
  - richer timing stats in `summary.json`
  - command marker `xplane_truth_capture/mark_event`
  - coarse automatic phase hints
  - extra VTOL/quadcopter/helicopter rotor, prop, collective, brake, joystick, weather, mass, and CG datarefs
- Tester plan:
  - fixed-wing no-wind baseline first if possible
  - Alia 250 VTOL run if available
  - quadcopter hover/yaw/translation run if available
  - helicopter optional and useful, but interpret carefully because collective/rotor datarefs can be aircraft-specific
  - use standard ZIP, not RAR
- New report: `report_v6.md`.

## 2026-04-30

### Second XPlaneTruthCapture Run Received

- Uploaded archive: `/home/alireza/20260430-004517Z.rar`.
- Extracted analysis folder: `/tmp/xplane-heli-capture-20260430/20260430-004517Z/`.
- Capture identity:
  - XPlaneTruthCapture `v0.1.2`
  - X-Plane 11.55r2, XPLM 303
  - Windows
  - Laminar Sikorsky S-76C helicopter
  - capture mode `every_frame`
- Bundle quality:
  - `3328` rows
  - `0` rows dropped
  - `149` requested datarefs
  - no user markers
  - aircraft reload at frame `3217`
- Segments after analyzer v0.1.3:
  - segment 0: frames `1-3217`, `138.098 s`
  - segment 1: frames `3218-3328`, `4.586 s`, post-reload static ground state
- Timing evidence:
  - median frame period about `0.04274 s`
  - median FPS about `23.4`
  - one reload gap about `46.3 s`
- Motion evidence:
  - not a real hover run
  - high-speed helicopter flight and crash/reload behavior
  - useful for rotorcraft dataref coverage, acceleration signs, and timing reset handling

### New Sensor Evidence

- Local velocity again matches finite differences of local position.
- True track derived from local velocity again matches X-Plane `hpath`.
- Magnetic heading relationship remains internally consistent:
  - `psi + magnetic_variation` matches `mag_psi` within small error.
- XP11 airspeed behavior confirmed again:
  - raw indicated airspeed is knots
  - raw true airspeed behaves like m/s
- X-Plane quaternion norm is valid, but direct MAVLink forwarding remains unproven.
- Strong accelerometer candidate from fixed-wing and helicopter captures:
  - `xacc = -g_axil * g`
  - `yacc = +g_side * g`
  - `zacc = -g_nrml * g`
- Current px4xplane likely has `yacc` sign wrong because it negates all three axes.
- `zacc` sign is very strong; `xacc/yacc` still need marked forward/lateral tests before final production claim.

### XPlaneTruthCapture v0.1.3

- Created commit: `5a35fd8 Release truth capture v0.1.3`.
- Created tag: `v0.1.3`.
- Pushed commit and tag to GitHub.
- Release workflow passed for Windows, macOS, and Linux.
- Release: https://github.com/alireza787b/xplane-truth-capture/releases/tag/v0.1.3
- Published assets:
  - `XPlaneTruthCapture-Windows.zip`
  - `XPlaneTruthCapture-macOS.zip`
  - `XPlaneTruthCapture-Linux.zip`
  - SHA256 files
- v0.1.3 changes:
  - segmented sim duration and sim-time reset count
  - final `Log.txt` copy at stop
  - analyzer segment outputs
  - analyzer array-dataref stats
  - viewer dark/light mode
  - viewer frame/sim-time/host-time X axis
  - viewer sim-reset warning
  - minimal footer with repo and `Alireza787b` LinkedIn link

### Plan Update

- Move next to implementation, but start with test/replay foundation.
- First px4xplane coding slice:
  - `CaptureReplaySource` from truth-capture CSV
  - unit tests for transforms and MAVLink units
  - zero-initialized MAVLink messages
  - stable `HIL_GPS.id`
  - true COG from local velocity
  - IAS conversion to cm/s
  - `HIL_STATE_QUATERNION` acceleration in mG
  - candidate accel mapping behind tests
  - patchy accel calibration/noise/filtering disabled by default

## 2026-05-04

### First px4xplane Test Binary

- Started implementation after the May 3 Alia/XP12 truth-capture analysis.
- Built first Linux test package:
  - `px4xplane-linux-v3.4.3-test010.zip`
  - SHA256 `c2604dbc5b98933ad8244f365f747745c08e5a826b5b67a7034de7893cbab74e`
- Staged plugin folder:
  - `build/lin/Release/px4xplane/`
- X-Plane binary:
  - `build/lin/Release/px4xplane/64/lin.xpl`
- Build command passed:
  - `cmake --build build --config Release --parallel 2`
- Verified exported entry points:
  - `XPluginStart`
  - `XPluginStop`
  - `XPluginEnable`
  - `XPluginDisable`
  - `XPluginReceiveMessage`

### Implemented In Test010

- Version set to `3.4.3`, phase `Sensor Contract Test`, build `010`.
- `HIL_SENSOR` accelerometer candidate changed to:
  - `xacc = -g_axil * 9.80665`
  - `yacc = +g_side * 9.80665`
  - `zacc = -g_nrml * 9.80665`
- Disabled accelerometer auto-calibration by default.
- Disabled vibration/noise injection by default.
- Zero-initialized `HIL_GPS` and `HIL_STATE_QUATERNION` payloads.
- Set stable `HIL_GPS.id = 0`.
- Changed GPS COG to true velocity-derived track and hold-last-valid at low speed.
- Kept GPS velocity source as X-Plane local velocity mapped to NED.
- Fixed `HIL_STATE_QUATERNION` velocity, airspeed, and acceleration units.
- Added `[BRIDGE_DIAG]` diagnostic lines to X-Plane `Log.txt`.
- Added menu actions:
  - `Reload Config`
  - `Diagnostics: On/Off`
- Added diagnostics status to the in-plugin data window.
- Added missing `<cstdint>` include in `ConnectionManager.h`.
- Added missing standard X-Plane plugin callbacks.

### Build Notes

- Canonical Linux build path is CMake, not `Makefile.linux`.
- Required setup:
  - `git submodule update --init --recursive`
  - Ubuntu/Debian package `libgl1-mesa-dev`
- `ldd` outside X-Plane reports `libfmod.so.13` and `libfmodstudio.so.13` as missing through SDK stubs; validate load behavior inside real X-Plane with `Log.txt`.

### Next Evidence Needed

- Test Alia first with the packaged binary.
- Collect:
  - X-Plane `Log.txt`
  - PX4 `.ulg`
  - PX4 console warnings
  - X-Plane version, OS, FPS, flight-models-per-frame setting
- Primary validation questions:
  - plugin load/connect path
  - EKF2 accel bias behavior
  - vertical velocity behavior
  - compass/GPS glitches
  - bridge message rates vs FPS
  - diagnostic acceleration/velocity/heading sanity
- New report: `report_v7.md`.

### XPlaneTruthCapture v0.1.4 Follow-up

- Created commit: `3ebdc53 Release truth capture v0.1.4`.
- Created tag: `v0.1.4`.
- Pushed commit and tag to GitHub.
- Release workflow passed for Windows, macOS, and Linux.
- Release: https://github.com/alireza787b/xplane-truth-capture/releases/tag/v0.1.4
- Published assets:
  - `XPlaneTruthCapture-Windows.zip`
  - `XPlaneTruthCapture-macOS.zip`
  - `XPlaneTruthCapture-Linux.zip`
  - SHA256 files
- Added `TEST_PROTOCOL.md` for controlled tester runs.
- Added more analyzer `derived.csv` columns for local position/velocity and track checks:
  - `local_y`
  - `local_vx_mps`
  - `local_vz_mps`
  - `true_track_deg`
  - `mag_track_deg`
  - `mag_yaw_deg`
  - `mag_variation_deg`
- Local validation passed:
  - analyzer syntax check
  - analyzer rerun on helicopter capture
  - CMake build
  - staged plugin contains `TEST_PROTOCOL.md`
- Recommendation:
  - ask tester to use `v0.1.4`
  - start px4xplane implementation in parallel
  - no more logger changes needed before the next controlled test
- New report: `report_v8.md`.

## 2026-05-02

### XP12 Alia Short Capture Received

- Uploaded archive: `/home/alireza/20260501-205224Z.zip`.
- Extracted analysis folder: `/tmp/xplane12-short-capture-20260501/20260501-205224Z/`.
- Capture identity:
  - XPlaneTruthCapture `v0.1.2`
  - X-Plane `12.2.1-r1`, build `122103`, XPLM `412`
  - Windows
  - Laminar/BETA Technologies `ALIA-250.acf`
  - capture mode `every_frame`
- Bundle quality:
  - `671` rows
  - `0` rows dropped
  - `33.67 s` sim duration
  - `63.38 s` host duration
  - one generic user marker at frame `401`
  - already airborne at capture start
  - no hover-like frames by current detector
- Dataref coverage:
  - `149` requested datarefs
  - only optional missing refs:
    - `sim/aircraft/weight/acf_m_total`
    - `sim/weather/aircraft/wind_speed_msc`
    - `sim/flightmodel/engine/ENGN_rpm`
  - XP12 `Log.txt` showed expected replacement warnings for older XP11-style refs, confirming the alias/version-drift problem remains real.

### XP12 Alia Sensor and Timing Evidence

- Local velocity again matches finite differences of local position:
  - `local_vx` p95 error about `0.0246 m/s`
  - `local_vy` p95 error about `0.0047 m/s`
  - `local_vz` p95 error about `0.0186 m/s`
- NED velocity mapping remains supported:
  - `vn = -local_vz`
  - `ve = local_vx`
  - `vd = -local_vy`
- Magnetic heading consistency remains strong:
  - `psi + magnetic_variation - mag_psi` median about `-0.0066 deg`
  - p95 about `0.0797 deg`
- Airspeed behavior matches XP11 evidence:
  - raw indicated airspeed equals cockpit IAS in knots
  - raw true airspeed behaves like `m/s`
  - IAS can go negative in Alia low-speed/vertical phases, so it is not a clean hover truth source.
- Quaternion norm is valid, but basis/body conversion for PX4 still needs implementation proof.
- Acceleration sign-fit again supports:
  - `xacc = -g_axil * g`
  - `yacc = +g_side * g`
  - `zacc = -g_nrml * g`
- This is now third-capture support for the candidate accel mapping and first evidence from XP12/Alia.
- Timing nuance:
  - median callback interval about `0.104 s`
  - `frame_rate_period` constant about `0.05025 s`
  - `xplm_cycle_number` and flight-loop counter advanced by exactly `2` between captured rows
  - bridge design must schedule sensor output explicitly instead of assuming one X-Plane callback equals one estimator sample.

### XPlaneTruthCapture v0.1.5

- Created commit: `37ae182 Release truth capture v0.1.5`.
- Created tag: `v0.1.5`.
- Pushed commit and tag to GitHub.
- Release workflow passed for Windows, macOS, and Linux.
- Release: https://github.com/alireza787b/xplane-truth-capture/releases/tag/v0.1.5
- Published assets:
  - `XPlaneTruthCapture-Windows.zip`
  - `XPlaneTruthCapture-macOS.zip`
  - `XPlaneTruthCapture-Linux.zip`
  - SHA256 files
- v0.1.5 additions:
  - `config/marker_plan.txt`
  - planned marker command
  - planned marker plus advance command
  - next/previous planned marker commands
  - reload marker plan menu action
  - marker ID/name/description/source/index fields in `events.jsonl`
  - marker plan copy inside each run folder
  - viewer event overlay show/hide
  - viewer marker-span range selection between consecutive markers
  - docs and `TEST_PROTOCOL.md` updated for planned test-card style runs
- Deliberate limitation:
  - no full in-sim marker text editor yet
  - config-driven marker plans are preferred for the next validation run because they are reproducible, source-controllable, and simpler for testers.

### Updated Recommendation

- Ask the next serious tester to use `v0.1.5`, not `v0.1.4`.
- Do not repeat random short runs unless they cover a missing aircraft class.
- Priority run is still a controlled VTOL sequence with planned markers:
  - ground static
  - lift/takeoff
  - hover start/end
  - yaw left/right
  - translate forward/back/right/left
  - climb/descent
  - landing
  - stop
- Proceed with px4xplane implementation foundation in parallel:
  - truth-capture replay source
  - transform/unit tests
  - MAVLink zero-init and stable IDs
  - true GPS COG from local velocity
  - IAS conversion
  - `HIL_STATE_QUATERNION` accel mG conversion
  - candidate accel mapping behind tests
  - structured bridge logging aligned with truth-capture event markers
- New report: `report_v9.md`.

## 2026-05-04

### May 3 VTOL Capture Received

- Uploaded archive: `/home/alireza/20260503-231859Z.zip`.
- Extracted analysis folder: `/tmp/xplane-vtol-capture-20260503/20260503-231859Z/`.
- Capture identity:
  - XPlaneTruthCapture `v0.1.5`
  - X-Plane `12.2.1-r1`, build `122103`, XPLM `412`
  - Windows
  - Laminar/BETA Technologies `ALIA-250.acf`
  - capture mode `every_frame`
- Bundle quality:
  - `3925` rows
  - `0` rows dropped
  - `177.39 s` sim duration
  - `178.20 s` host duration
  - analyzer issues: none
  - missing required datarefs: none
- Marker quality:
  - planned marker system was present
  - tester repeated `GROUND_STATIC` three times
  - marker labels are not usable as maneuver truth
  - phase inference used throttle, ground contact, velocity, attitude, AGL, and vertical speed instead

### May 3 VTOL Evidence

- Inferred sequence:
  - throttle-up began around `50.8 s`
  - throttle exceeded `0.5` around `67.7 s`
  - off-ground around `70.0 s`
  - speed exceeded `10 m/s` around `96.1 s`
  - speed exceeded `40 m/s` around `148.4 s`
  - final segment became an aggressive airborne descent/fall, not a landing
- Not a clean hover test:
  - hover candidate rows: `0`
  - no yaw input observed
  - final attitude reached roll `-119.47 deg`, pitch `-37.04 deg`
- Local velocity remains validated by finite differences:
  - `local_vx` p95 error about `0.058 m/s`
  - `local_vy` p95 error about `0.022 m/s`
  - `local_vz` p95 error about `0.045 m/s`
- Heading/track remains clean:
  - magnetic heading relation p95 error about `0.047 deg`
  - true track from local velocity vs `hpath` p95 error about `0.059 deg` when speed > `5 m/s`
- Accel sign-fit on clean airborne rows again supports:
  - `xacc = -g_axil * g`
  - `yacc = +g_side * g`
  - `zacc = -g_nrml * g`
- May 3 acceleration RMSE:
  - current `[-g_axil,-g_side,-g_nrml]`: X `0.0483`, Y `0.2163`, Z `0.0113`
  - candidate `[-g_axil,+g_side,-g_nrml]`: X `0.0483`, Y `0.0329`, Z `0.0113`
- This makes px4xplane's current `g_side` negation a high-confidence defect.

### Flight Models Per Frame Conclusion

- X-Plane `loop_counter` and `xplm_cycle_number` advanced by `2` between every captured row in:
  - XP11 helicopter
  - XP12 short Alia
  - XP12 May 3 Alia
- X-Plane SDK states flight-model iterations above one are opaque to plugins.
- Therefore, the observed `+2` counter pattern is not evidence that px4xplane can receive two useful plugin-visible sensor updates per frame.
- Raising X-Plane "flight models per frame" may improve X-Plane physics stability for agile/light aircraft, but should not be used as a data-rate solution.
- Recommended user guidance:
  - keep setting at least `2`
  - try `3-4` for agile VTOLs only if FPS stays healthy
  - do not max it by default
  - px4xplane should not override it silently
- Rewrite must handle actual plugin-visible FPS, commonly `20-30 Hz`, with an explicit sensor scheduler/resampler.

### Implementation Readiness

- Ready to start implementation with replay tests first.
- Guaranteed fix list:
  - accel Y sign
  - `HIL_STATE_QUATERNION` accel mG conversion
  - IAS/TAS conversions in state messages
  - true COG from local velocity
  - `HIL_GPS` zero-init and stable `id`
  - zero-initialized MAVLink messages
  - explicit low-FPS sensor scheduler
  - remove patchy accel calibration/noise/filtering from default path
- Still needed before final production claim:
  - correctly marked Alia/VTOL hover/yaw/translation/landing run
  - tester note or screenshot for flight-model-per-frame setting
  - optional A/B timing run at setting `2` vs `4`
- New report: `report_v10.md`.

## 2026-05-06

### Full X-Plane 12 Alia Validation Run Received

- Evidence folder: `/home/alireza/alia-sitl1`.
- Scenario:
  - X-Plane 12 Alia
  - model calculations per frame: `6`
  - takeoff to about `100 m`
  - automatic forward transition
  - circle/loiter segment
  - RTL return
  - back-transition
  - landing and disarm
- Outcome:
  - PX4 completed the full workflow.
  - XPlaneTruthCapture recorded `43,764` frames over about `518.2 s`.
  - Recorded truth rows had `0` dropped rows and `0` sim-time resets.
  - Mean callback rate was about `84.5 Hz`; p50 FPS about `83 Hz`; minimum
    instantaneous FPS about `19.9 Hz`.
  - X-Plane bridge diagnostics remained stable.
  - PX4 CLI showed a vertical velocity unstable warning after landing/disarm/log
    close, not during the flight workflow.

### px4xplane v3.4.3 Follow-Up Slice

- Kept the current sensor contract as the baseline:
  - `xacc = -g_axil * g`
  - `yacc = +g_side * g`
  - `zacc = -g_nrml * g`
  - GPS velocity from X-Plane local velocity
  - GPS course over ground from true local velocity track
  - `HIL_STATE_QUATERNION` acceleration in milli-g
- Added bridge health diagnostics:
  - effective sent Hz for sensor/GPS/state/RC
  - frame period min/p50/p95/max
  - dropped/late/missed send counters
  - timestamp monotonicity counters
  - clear warning when configured target rates exceed achievable frame rate
- Added an injectable dataref provider seam to start separating sensor generation
  from direct X-Plane reads.
- Added `tools/replay_truth_capture.py` as a deterministic core-contract replay
  tool for XPlaneTruthCapture folders/zips.
- Added `tools/analyze_ulog_estimator.py` for ULog estimator topic/event-window
  summaries. It ran on `/home/alireza/alia-sitl1/05_17_33.ulg` using temporary
  `pyulog`; the log included requested estimator topics except `commander_state`.
- Removed outgoing HIL_SENSOR timestamp jitter so same-frame MAVLink timestamps
  remain globally monotonic.
- Fixed waiting-socket disconnect cleanup so cancelling PX4 connection wait closes
  sockets and resets status.
- Reduced release-default debug log noise while keeping compact diagnostics on
  at a `2 s` interval.
- Cleaned and conservatively tuned `5020_xplane_alia250`, separating bridge
  contract settings from aircraft-performance tuning.
- Added `docs/ALIA_XPLANE12_TEST.md` as the next test run card.
- Updated `CHANGELOG.md` with the `3.4.3` validation-follow-up slice.

### XPlaneTruthCapture v0.1.6 Follow-Up

- Added active recording feedback:
  - menu state changes to `Recording Active`
  - Start is disabled while recording
  - Stop is enabled only while recording
- Added a configurable minimal overlay with elapsed time, frame count, row count,
  dropped rows, and next planned marker.
- Added config keys:
  - `overlay_enabled`
  - `overlay_x`
  - `overlay_y_from_top`
- Fixed marker behavior:
  - generic `mark_event` no longer consumes the first planned marker when a
    marker plan is loaded.
- Updated README and test protocol with the new recording indication behavior.

### PX4 Fork State

- The local PX4 fork used during the run was referenced in CLI logs as
  `/home/alireza/PX4-Autopilot-Me`, branch/workflow `px4xplane-sitl`.
- That worktree was not present in the current local filesystem.
- Canonical Alia param source for sync remains:
  `config/px4_params/5020_xplane_alia250`.
- Expected fork destination after restoring/cloning the fork:
  `PX4-Autopilot-Me/ROMFS/px4fmu_common/init.d-posix/airframes/5020_xplane_alia250`.

### Next Workflow

- Produce Windows packages for px4xplane and XPlaneTruthCapture and place them in
  `/home/alireza` for the next test.
- Push/tag XPlaneTruthCapture `v0.1.6` so CI can produce official Windows/macOS
  release assets.
- Push/tag px4xplane `v3.4.3` after final package verification.
- Next implementation slices after the retest:
  - prove or replace quaternion conversion
  - clean barometer and magnetometer contracts
  - add structured bridge session logs
  - move from diagnostics-only low-FPS handling to a tested scheduler/resampler
  - restore/sync the PX4 fork and resume PR cleanup after the bridge evidence is stable.

### px4xplane v3.4.4 CI Hotfix

- The first public `v3.4.3` release tag built and uploaded Linux/macOS assets,
  but the GitHub Actions Windows job failed under MSVC.
- Root cause: the new diagnostics used `std::min`/`std::max` in
  `src/px4xplane.cpp`; MSVC's Windows headers exposed `min`/`max` macros that
  rewrote those calls.
- Fix: protected the affected calls with `(std::min)` / `(std::max)`.
- Local verification after the fix:
  - Linux CMake build passed.
  - MinGW Windows CMake build passed.
  - Replay regression test passed.
- This is a build portability hotfix only; no sensor-contract or Alia tuning
  change was made relative to `v3.4.3`.

### px4xplane v3.4.5 Alia Comment Cleanup

- Cleaned the tail troubleshooting notes in `5020_xplane_alia250` after spotting
  stale numeric hints from older tuning iterations.
- The comments now reference the current baseline values for front transition,
  back transition, NPFG/loiter, and TECS damping.
- No runtime tuning value changed in this cleanup; it keeps the public
  `px4xplane` release, local test package, and PX4 fork airframe file aligned.

## 2026-05-07

### Alia Retest Regression Triage

- New evidence folder: `/home/alireza/alia-sitl2`.
- Available evidence in this environment: two ULogs only:
  - `19_07_20.ulg`
  - `19_10_42.ulg`
- X-Plane `Log.txt` and XPlaneTruthCapture data were not available in the folder.
- `19_07_20.ulg` is not an Alia log: `SYS_AUTOSTART=5010`.
- `19_10_42.ulg` is Alia (`SYS_AUTOSTART=5020`) and matches the successful
  `alia-sitl1/05_17_33.ulg` on key MC/VTOL/FW parameters that were logged.
- The previous sync was still flawed:
  - The PX4 fork had been the user's tested operating source.
  - The plugin repo airframe file contained unproven later tuning values.
  - Syncing the plugin repo file into the PX4 fork treated stale repo state as
    the source of truth.
- Corrected policy:
  - Keep the successful ULog parameter values as the baseline.
  - Revert unproven FW/TECS/NPFG/loiter changes until controlled retests prove
    them.
  - Do not infer an Alia tuning failure from a `SYS_AUTOSTART=5010` ULog.
- User note: the GPU PC did not have git available, so the airframe file was
  updated by manual copy. That is acceptable only if PX4 SITL parameters are
  reset afterward; otherwise saved parameters can mask or override the edited
  airframe defaults.
- XPlaneTruthCapture v0.1.6 local Windows package failure found:
  - The `/home/alireza` zip used a local MinGW build.
  - `win.xpl` depended on `libgcc_s_seh-1.dll` and `libstdc++-6.dll`.
  - Those DLLs were not packaged, so X-Plane could fail to load the plugin before
    the menu appeared.
- TruthCapture fix:
  - v0.1.7 statically links the MinGW GCC runtime for local Windows builds.
  - Official GitHub release assets remain preferred for Windows when available.

### px4xplane Config Safety Slice

- Completed the first approved offline implementation slice without changing
  Alia params, sensor signs/units, quaternion handling, or prop-brake policy.
- Config now loads at plugin startup and clears stale actuator mappings on every
  reload.
- Added tested actuator safety helpers for finite/clamped scaling.
- Added stale `HIL_ACTUATOR_CONTROLS` detection; configured actuator datarefs
  are zeroed if PX4 actuator input stops arriving.
- Inbound MAVLink receive now drains multiple bounded chunks per X-Plane frame
  to reduce low-FPS backlog risk.
- Added `tools/validate_config.py` plus Python tests for active section,
  channel format, data type, range, array-index, and prop-brake validation.
- Optional CTest now runs the C++ actuator-safety test, config validator CLI,
  and Python offline-tool tests.
- Updated custom-airframe docs and Mixing tab help to clarify reload vs
  reconnect behavior and supported runtime data types.
- Verification passed:
  - Python validator/replay unit tests
  - config validator CLI
  - py_compile for offline tools
  - CMake configure/build for offline tests
  - CTest
  - Linux plugin build
  - MinGW Windows plugin build
  - Windows plugin import inspection
- New report: `docs/reports/report_v15.md`.

### px4xplane Runtime Config Validation UI Slice

- Completed the second approved offline implementation slice without changing
  Alia params, sensor signs/units, quaternion handling, or prop-brake policy.
- Added runtime config validation state in `ConfigManager`.
- Active X-Plane-side mapping validation now checks parsed actuator channels,
  data types, ranges, array indices, and whether configured datarefs exist in
  the currently loaded X-Plane aircraft.
- Missing `config.ini` now surfaces as an explicit validation error after load
  or reload.
- Added `Advanced` menu with:
  - validation status
  - `Validate Config`
  - `Reload and Validate Config`
  - `Bridge Diagnostics: On/Off`
  - `Show Docs Location`
- Moved diagnostics out of the normal top-level menu to reduce normal-user
  noise while keeping advanced access.
- Connection and Mixing tabs now show config validation status, with the Mixing
  tab showing the first three validation messages.
- Added `docs/index.md` as the stable repo docs entry point.
- Hardened plugin menu/window cleanup on menu rebuild and plugin stop.
- Verification passed:
  - Python validator/replay unit tests
  - config validator CLI
  - py_compile for offline tools
  - CTest
  - Linux plugin build
  - MinGW Windows plugin build
  - Windows plugin import inspection
- New report: `docs/reports/report_v16.md`.

### px4xplane Config Schema Metadata Slice

- Completed a behavior-preserving schema metadata slice for `config.ini`.
- Added `config/config_schema.json` with current global config fields,
  supported actuator mapping types, numeric bounds, groups, and reload policy.
- `tools/validate_config.py` now loads the schema and validates global field
  types/ranges in addition to airframe channel mappings.
- Added `--schema` and `--list-fields` to the config validator.
- CMake now copies `config_schema.json` beside `config.ini` in packaged plugin
  folders.
- Added `docs/developer/config-schema.md` and linked it from `docs/index.md`.
- Updated custom-airframe docs with explicit live-reload vs
  reconnect-before-flight guidance.
- Verification passed:
  - JSON schema parse with `jq`
  - Python validator/replay unit tests
  - config validator CLI and field listing
  - py_compile for offline tools
  - CTest
  - Linux plugin build
  - MinGW Windows plugin build
  - whitespace check
- No Alia params, sensor contract, actuator behavior, prop-brake behavior, PX4
  fork files, or XPlaneTruthCapture files changed in this slice.
- New report: `docs/reports/report_v17.md`.

### px4xplane Static Config Editor Prototype

- Added the first schema-backed static config editor:
  - `docs/config-editor.html`
  - `docs/assets/config-editor.js`
- Runtime remains `config.ini`; JSON schema is metadata for validation/editor
  UX, not a runtime replacement.
- The editor can import `config.ini`, optionally import `config_schema.json`,
  edit global fields, add/rename/remove airframes, edit `autoPropBrakes`, edit
  channel mappings, validate, preview, and export a clean `config.ini`.
- The editor does not write into X-Plane, PX4, or a running simulator.
- CMake packages the editor under `px4xplane/docs/`.
- Added `tests/test_config_editor.js` and wired it into optional CTest when
  `node` is available.
- README, docs index, and config-schema docs now reference the editor.
- Verification passed:
  - Node editor test and syntax check
  - Python validator/replay unit tests
  - config validator CLI
  - py_compile for offline tools
  - CMake reconfigure for offline tests
  - CTest
  - Linux plugin build
  - MinGW Windows plugin build
  - whitespace check
- No Alia params, bridge runtime behavior, PX4 fork files, or
  XPlaneTruthCapture files changed in this slice.
- New report: `docs/reports/report_v18.md`.

### px4xplane Config Editor Menu Link Slice

- Added `Advanced > Show Config Editor Location` to the X-Plane plugin menu.
- The menu action writes the packaged editor path to X-Plane `Log.txt`:
  `px4xplane/docs/config-editor.html`.
- The plugin status line reports that the editor path was written.
- Deliberately avoided automatic browser/file launching in this slice; that
  needs platform-specific handling and tests.
- Verification passed:
  - CTest
  - Linux plugin build
  - MinGW Windows plugin build
  - whitespace check
- No Alia params, bridge runtime behavior, PX4 fork files, or
  XPlaneTruthCapture files changed in this slice.
- New report: `docs/reports/report_v19.md`.

### px4xplane v3.4.8 Release Build Recovery And Test Kit

- `v3.4.7` was tagged and GitHub created a release, but the Windows MSVC
  release job failed while Linux/macOS assets uploaded successfully.
- Root cause was a Windows `max` macro collision with
  `std::numeric_limits<uint64_t>::max()` in `src/MAVLinkManager.cpp`.
- Fixed the portability issue by:
  - adding `NOMINMAX` to Windows compile definitions
  - changing the vulnerable call to `(std::numeric_limits<uint64_t>::max)()`
- Bumped to `v3.4.8` instead of rewriting the already-published `v3.4.7` tag.
- Local Linux and MinGW Windows builds passed after the fix.
- Created local next-test kit:
  - `/home/alireza/px4xplane-v3.4.8-next-test-kit-20260507.zip`
- Kit contents:
  - `px4xplane-windows-v3.4.8-test016-20260507.zip`
  - `XPlaneTruthCapture-Windows-v0.1.7.zip`
  - `5020_xplane_alia250_v3.4.8_test016`
  - `README_NEXT_TEST.md`
- Confirmed `/home/alireza/PX4-Autopilot-Me` is clean on
  `px4xplane-sitl` and its Alia airframe hash matches the packaged/source
  baseline.
- No Alia tuning, sensor-contract, actuator mapping, prop-brake policy, or
  XPlaneTruthCapture code change was made in this recovery slice.
- New report: `docs/reports/report_v20.md`.

### px4xplane v3.4.9 Alia Vertical Tune And Baro Source Fix

- Analyzed `/home/alireza/alia-test2.zip` plus the pasted PX4 terminal log.
- Confirmed the run was valid Alia evidence with `SYS_AUTOSTART=5020`, full
  takeoff, front transition, RTL, back transition, landing, disarm, and ULog
  close.
- Truth capture quality was strong: `42,344` frames, zero dropped rows, no
  sim-time resets, and about `77.5 Hz` mean callback rate.
- Main remaining flight issue is real vertical dynamics:
  - fixed-wing climb rate swung about `-12.3` to `+12.6 m/s`
  - worst 10 s fixed-wing altitude swing was about `61.7 m`
  - TECS saturated throttle and pitch while airspeed remained accepted
- Baro switch root cause was clarified:
  - PX4 simulator_mavlink publishes two simulated baro uORB instances from one
    incoming `HIL_SENSOR` stream
  - airframes were setting priorities without seeding the simulated device IDs
  - PX4 could therefore auto-create both baros at default priority `50`
- Changes prepared:
  - bump px4xplane to `v3.4.9` build `017`
  - seed simulated baro IDs and disable baro1 in all X-Plane PX4 airframes
  - relax `EKF2_BARO_NOISE` to `0.05`
  - damp Alia TECS and back-transition parameters
  - add prop-brake hysteresis and clamp negative IAS before diff-pressure
- Updated `docs/ALIA_XPLANE12_TEST.md` with the v3.4.9 parameter sanity list
  and the requirement to run `distclean` or otherwise reset SITL parameters
  after replacing airframe defaults.
- New report: `docs/reports/report_v21.md`.

### px4xplane v3.4.10 Alia Regression Recovery

- Analyzed `/home/alireza/alia-test3.zip` and `/home/alireza/alia-test4.zip`.
- Confirmed both runs used `SYS_AUTOSTART=5020` and `px4xplane v3.4.9`.
- Found the v3.4.9 Alia TECS/back-transition tuning degraded lateral tracking
  and landing behavior despite reducing some vertical-rate extremes.
- Found the test4 high-accelerometer-bias warning came from a fresh `distclean`
  simulated accel calibration state, not a new bridge sign/axis failure.
- Kept the baro device-ID/priority fix, but corrected `EKF2_BARO_NOISE` from
  overconfident `0.05 m` to `1.0 m` across X-Plane airframes.
- Seeded Alia `CAL_ACC0` simulated accel offsets from the successful baseline.
- Reverted the unproven Alia v3.4.9 TECS, throttle, back-transition, and RTL
  descent tuning to the better-tracking baseline.
- TruthCapture showed hover props still windmilled around `33-35 rad/s` in
  fixed-wing with throttle zero; prop brakes now enforce feather/zero-speed
  state continuously while braked.
- New report: `docs/reports/report_v22.md`.

### px4xplane v3.4.11 Alia Crash Recovery

- Analyzed `/home/alireza/alia-test5.zip` after the transition crash.
- Confirmed the terminal PX4 event was `Quad-chute triggered due to minimum
  altitude breach`, not an estimator/baro/airspeed failure.
- Found PX4 stayed in `TRANSITION_TO_FW`; no fixed-wing VTOL state appeared.
- Found the effective front-transition airspeed gate was about `51.0 m/s`
  because `VT_ARSP_TRANS=48` is scaled by `sqrt(WEIGHT_GROSS/WEIGHT_BASE)`,
  while test5 peaked around `50.6 m/s`.
- Found v3.4.10 prop-brake enforcement was unsafe: TruthCapture showed
  feathered/stopped/asymmetric lift props during recovery while PX4 commanded
  thrust.
- Disabled Alia auto prop braking by default and made future opt-in braking
  dwell-gated, all-motor gated, immediate-release, and non-failure-based by
  default.
- Narrowly reduced Alia front-transition gates to `VT_ARSP_TRANS=46`,
  `VT_F_TRANS_DUR=45`, and `VT_F_TR_OL_TM=55`.
- New report: `docs/reports/report_v23.md`.

### px4xplane v3.4.12 Alia Fixed-Wing Tuning

- Analyzed `/home/alireza/alia-test6.zip`.
- Confirmed the v3.4.11 crash recovery worked: Alia reached fixed-wing, RTL,
  back-transition, landing, disarm, and log close without estimator warnings,
  baro switching, or quad-chute.
- Found the remaining fixed-wing issue is controller saturation, not a bridge
  sensor or throttle mapping bug:
  - TECS commanded pusher throttle above `0.99` for about `91%` of FW flight.
  - TruthCapture confirmed the X-Plane pusher throttle was actually `1.0`.
  - FW roll setpoint spent about `90%` of orbit above `25 deg` magnitude and
    flipped sign every roughly `17-19 s`.
  - X-Plane mass was about `3118 kg`, while PX4 was configured as `3500 kg`.
- Prepared v3.4.12 tuning:
  - match `WEIGHT_BASE`/`WEIGHT_GROSS` to `3120 kg`
  - smooth TECS altitude/energy response
  - lengthen/smooth NPFG and FW roll response
  - set back-transition expected deceleration to the measured `2.2 m/s^2`
- Fixed a local packaging gap found during zip validation: config/docs/airframe
  assets now refresh even when the plugin binary does not relink.
- Kept Alia prop braking disabled; generic opt-in prop-brake policy remains the
  systematic reusable bridge solution.
- New report: `docs/reports/report_v24.md`.

### px4xplane v3.4.13 Alia Test7 Orbit Recovery

- Analyzed `/home/alireza/alia-test7.zip`.
- Confirmed test7 used the intended v3.4.12 params and `SYS_AUTOSTART=5020`.
- Found the v3.4.12 lateral tune was too lazy:
  - `NPFG_PERIOD=70` was active.
  - FW loiter/RTL roll setpoint stayed near `+-30 deg` for more than `90%` of
    fixed-wing flight.
  - Loiter signed track error reached roughly `-1000 m` to `+900 m`.
- Confirmed throttle saturation was real:
  - TECS and pusher command were near `1.0` for most FW RTL.
  - TruthCapture confirmed the X-Plane pusher was actually full throttle.
- Clarified takeoff behavior:
  - Hover takeoff climb used `MPC_TKO_SPEED=1.5`, not the FW climb parameter.
  - The local MC takeoff setpoint plateaued around `62.9 m` before the external
    Auto Loiter command locked current altitude.
- Prepared v3.4.13:
  - `MPC_TKO_SPEED=3.0`
  - `FW_PSP_OFF=3.0`
  - lower FW bank limit and shorter NPFG period
  - `NAV_LOITER_RAD`/`RTL_LOITER_RAD=1200`
  - stronger roll-to-throttle compensation
- Reduced normal plugin log noise and documented packaged PX4 airframes as
  reference/install copies only.
- New report: `docs/reports/report_v25.md`.

### px4xplane v3.4.14 Alia Final Polish and Ehang Prep

- Analyzed `/home/alireza/alia-test8.zip`, the first broadly accepted Alia run.
- Confirmed v3.4.13 used `SYS_AUTOSTART=5020`, had no ULog dropouts, no
  in-flight estimator/baro/GPS warnings, and zero TruthCapture dropped rows.
- Found landing duration was mostly commanded descent time from about `165 m`
  AGL, not an estimator or bridge fault.
- Changed Alia RTL/landing to descend faster above `8 m` AGL, slow the final
  touchdown phase below `3 m`, and stop treating `300 m` as the RTL descent
  altitude.
- Increased Alia FW loiter/RTL radius to `1500 m` and restored `FW_R_LIM` to
  PX4's valid minimum of `35 deg`.
- Prepared Ehang 184 for first controlled validation with PX4-valid low jerk,
  explicit acceleration limits, smoother yaw/tilt limits, and a documented
  test card.
- Moved tracked root reports v12/v13 into `docs/reports` and removed a stale
  tracked Windows temp file.
- New report: `docs/reports/report_v26.md`.

### px4xplane v3.4.15 Alia Bank Recovery and Ehang Nav Tuning

- Analyzed `/home/alireza/evtol1.zip`, covering one low-FPS Alia startup, one
  v3.4.14 Alia flight, and one first Ehang 184 flight.
- Found the Alia regression was caused by v3.4.14 restoring `FW_R_LIM=35`:
  the aircraft spent about `45%` of fixed-wing time above `34 deg` actual bank
  and about `91%` of fixed-wing time at max throttle, with frequent elevator
  saturation and altitude loss. v3.4.13 had no high-bank fraction with
  `FW_R_LIM=22`.
- Restored `FW_R_LIM=22` and increased Alia `NAV_LOITER_RAD` and
  `RTL_LOITER_RAD` to `2000 m`.
- Re-enabled Alia lift-prop braking through the generic opt-in brake policy,
  gated by low command, dwell time, and airspeed, with X-Plane failure seizure
  still disabled.
- Updated Alia and Ehang `CAL_ACC0` seed offsets from the latest clean logs to
  avoid stale fresh-`distclean` calibration and low-FPS preflight bias warnings.
- Tuned Ehang 184 first-log defaults: slower horizontal cruise/max/manual
  speed, faster takeoff than v3.4.14, tighter `NAV_ACC_RAD`, and
  `LNDMC_Z_VEL_MAX=0.20`.
- Documented that QGC `MAV_CMD_DO_REPOSITION` was accepted in the Ehang log but
  not flown while PX4 stayed in Position mode; the next Ehang validation should
  use Orbit, mission/auto, RTL, and explicit mode checks.
- New report: `docs/reports/report_v27.md`.
