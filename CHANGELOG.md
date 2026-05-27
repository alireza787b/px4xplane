# Changelog

All notable changes to the PX4-XPlane plugin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [3.4.55] - 2026-05-27

### Changed

- Analyzed `cessna2.zip`: PX4 loaded the v3.4.54 Cessna params and produced
  nonzero `landing_gear_wheel` commands, but X-Plane loaded px4xplane
  `v3.4.53`, which did not include the Cessna channel 5 mapping. The runway
  steering failure was therefore a tested-package mismatch plus insufficient
  physical wheel path coverage, not proof that PX4 never commanded steering.
- Kept the Cessna wheel-controller output on `PWM_MAIN_FUNC6=440`, increased
  wheel-controller authority, and mapped Cessna channel 5 to both
  `sim/flightmodel2/gear/tire_steer_command_deg[0]` and
  `sim/flightmodel/parts/tire_steer_cmd[0]` while the bridge enables
  `override_wheel_steer`.
- Added Cessna landing flaps: PX4 control surfaces 4/5, `PWM_MAIN_FUNC7=205`,
  `PWM_MAIN_FUNC8=206`, and X-Plane flap mappings on channels 6/7. Landing
  configuration now deploys early, lowers landing airspeed, and extends flare
  timing to reduce the high-energy 90 kt touchdown seen in `cessna2`.
- Raised Cessna mission takeoff altitude to reduce immediate low-altitude
  turns after rotation.
- Simplified config-editor actuator mapping fields: scalar mappings show
  `scalar`; array mappings show bracket tokens outside a numeric-only index
  field; output ranges show brackets outside min/max fields. The left airframe
  list remains visible and only marks the active airframe.
- Made `ConfigManager::parseConfig()` clear prior actuator mappings itself, so
  future direct parser calls cannot leave stale channels even outside the normal
  `loadConfiguration()` path.
- Updated XPlaneTruthCapture to v0.1.8 with wheel-steering override and
  `sim/flightmodel/parts/tire_steer_cmd` capture coverage for runway debugging.

### Notes

- The next Cessna test must show `px4xplane: Version: v3.4.55` in X-Plane
  `Log.txt`; otherwise the bridge-side steering/flap changes are not installed.

## [3.4.54] - 2026-05-27

### Changed

- Analyzed `cessna1.zip`: the run used Cessna in the bridge, but PX4 did not
  map the `landing_gear_wheel` output to any actuator output and the bridge did
  not expose a dedicated Cessna nosewheel channel. This explains the runway
  takeoff and rollout centerline drift.
- Added Cessna nosewheel steering as `PWM_MAIN_FUNC6=440` and mapped px4xplane
  Cessna channel 5 to `sim/flightmodel2/gear/tire_steer_command_deg`.
- Seeded Cessna simulated accelerometer offsets from the values PX4 committed at
  disarm in `cessna1` instead of raising `EKF2_ABL_LIM` beyond PX4's declared
  parameter maximum.
- Softened Cessna roll-rate response and roll slew for a less jerky
  manned-aircraft feel while leaving the accepted pitch/TECS baseline mostly
  intact.
- Updated the no-flap autoland baseline: lower landing airspeed, earlier flare,
  configured touchdown timing, and a land-detector airspeed threshold consistent
  with the returned touchdown evidence.
- Made Cessna the packaged active config for this test slice and updated the
  Cessna test card, report index, and config editor version text.
- Adjusted the config editor layout so the left airframe list stays visible
  while the middle runtime/setup/actuator/camera sections are collapsible.

### Notes

- The `cessna1` ULog does not represent clean v3.4.54 parameters; it contains
  the temporary user-raised `EKF2_ABL_LIM=2.0` and older Cessna runway/landing
  values. Use `make px4_sitl_default distclean` before judging the next run.

---

## [3.4.53] - 2026-05-27

### Changed

- Tightened the config editor empty state and file-loading copy so users are
  directed to load `px4xplane/64/config.ini`; schema JSON is now clearly labeled
  as optional metadata and is rejected from the runtime-config picker.
- Reordered the editor workflow: active airframe first, inactive airframes in a
  collapsed group, global fields above the selected airframe, actuator mappings
  before camera presets, and no redundant description column.
- Kept per-field documentation links only where they are specific enough to be
  useful: actuator channel mappings and camera views. Generic field help stays
  in the `?` tooltip/reload-policy badge.
- Added a structured Cessna 172 runway-mission test card and packaged it with
  the plugin.
- Refreshed `5001_xplane_cessna172` as a clean first-pass runway takeoff,
  mission, and autoland baseline: current sensor contract, virtual pitot policy,
  current PX4 parameter bounds, runway takeoff, wheel controller, TECS/NPFG
  cleanup, and Cessna reference mass/speed values.
- Added report v65 and updated docs/index/custom-airframe guidance for the
  single runtime config source and current actuator dataref type support.

### Notes

- Alia remains the packaged default active airframe. Set
  `config_name = Cessna172` only when preparing the Cessna validation run.
- The Cessna defaults are an evidence-based initial baseline. Final C172 tuning
  still depends on returned ULog, X-Plane Log.txt, and XPlaneTruthCapture data.

---

## [3.4.52] - 2026-05-27

### Changed

- Added config-driven forward/down/chase camera presets for all packaged
  airframes while preserving the accepted QuadTailsitter camera offsets.
- Rebuilt the camera menu after airframe changes and released active custom
  camera control during airframe switches to avoid stale menu/index behavior.
- Replaced Advanced menu path logging with `Open Config Editor`,
  `Open Documentation`, and `Open GitHub Repository`; each uses OS-native
  opening with a Log.txt fallback if the platform blocks it.
- Clarified the config source of truth: the plugin reads only `64/config.ini`.
  `config_schema.json` is editor/validator metadata and is now packaged under
  `docs/` rather than next to the runtime INI.
- Upgraded the static config editor with automatic package loading when browser
  local-file policy allows it, dark mode, version metadata, footer links,
  inline field help, and structured camera-view editing.
- Expanded editor fallback validation so release configs no longer show false
  unknown-global-key warnings when the schema file is not manually imported.
- Matched JS/Python camera validation to runtime zoom bounds (`0 < zoom <= 4`).
- Added report v64 and refreshed docs/index/test card guidance for v3.4.52.

### Notes

- No PX4 airframe parameters, QuadTailsitter ACF geometry, sensor contract, or
  accepted qtail26/qtail27 flight-control tuning changed in this slice.

---

## [3.4.51] - 2026-05-27

### Changed

- Analyzed `qtail27.zip`: the run loaded the intended v3.4.50 QuadTailsitter
  params, flew cleanly until manual RTL, then descended to the configured
  `RTL_DESCEND_ALT=75 m` before auto back-transition. The high-energy recovery
  consumed that margin and caused roll failure/crash after MC handoff.
- Raised QuadTailsitter manual-RTL margins to `RTL_RETURN_ALT=180` and
  `RTL_DESCEND_ALT=150` while leaving the accepted qtail26/qtail27 bridge, ACF,
  forward transition, FW guidance, and back-transition timing unchanged.
- Added config-driven camera views and a `Camera Views` menu. QuadTailsitter now
  defines nose/FPV, belly/down, and chase views in `config.ini`; the commands are
  generic (`px4xplane/camera/view_1..view_8`) so future airframes can add views
  without C++ hardcoding or numpad quick-look shortcuts.
- Added report v63 and updated the QuadTailsitter test card/aircraft README for
  v3.4.51 validation.

### Notes

- This release does not retune `VT_B_TRANS_DUR`, `VT_B_DEC_MSS`, or the ACF.
  qtail27 points at insufficient RTL altitude margin for manual RTL, not a
  broad aircraft-physics regression.

---

## [3.4.50] - 2026-05-26

### Changed

- Closed the QuadTailsitter qtail26 acceptance pass as the new validation
  baseline: full mission, front transition, fixed-wing return,
  back-transition, landing, disarm, and log close completed with clean bridge
  and estimator behavior.
- Kept the qtail26 X-Plane ACF, sensor contract, body-axis pitot, rotor cant,
  and back-transition timing unchanged after independent review found no
  evidence for another aircraft-physics or bridge rewrite.
- Made a conservative final PX4 airframe polish: `MPC_Z_VEL_D_ACC 0.5 -> 0.6`,
  `FW_AIRSPD_TRIM 28 -> 27`, `FW_AIRSPD_MAX 38 -> 36`, and
  `FW_THR_TRIM 0.15 -> 0.14`.
- Added QuadTailsitter aircraft-selection icons, aircraft-local quick-look
  presets for nose/FPV, belly/down, and rear engineering views, plus an
  aircraft README explaining the visual scope.
- Added report v62 and updated the QuadTailsitter closure test card for the
  qtail26 evidence package and v3.4.50 sanity checks.

### Notes

- `VT_B_TRANS_DUR` and `VT_B_DEC_MSS` stay at the qtail26-proven values
  (`5.0 s` and `1.0 m/s/s`). Extending the schedule was deliberately not
  shipped because qtail26 showed the recovery climb is dominated by stored
  forward-speed energy after transition, not by a missing timeout increase.
- The polished eVTOL cockpit/panel remains a separate Plane Maker-authored
  visual slice; this package only adds safe source-controlled quick-look/icon
  assets.

---

## [3.4.49] - 2026-05-26

### Changed

- Retuned QuadTailsitter after qtail25 completed a full safe mission return but
  still showed fixed-wing yaw/roll wobble, a large back-transition recovery
  climb, transient attitude-failure warnings, and early MC yaw oscillation
  during descent.
- Reduced QuadTailsitter fixed-wing entry energy from `29` to `28 m/s`, lowered
  the transition airspeed to `22 m/s`, softened FW roll/yaw differential-thrust
  coupling, and increased NPFG/loiter spacing for smoother mission turns.
- Extended the tailsitter back-transition schedule from `4` to `5 s` and
  reduced the mission deceleration estimate while preserving the successful
  qtail25 aircraft and bridge baseline.
- Added tailsitter-specific failure-detector angle/dwell tolerances without
  disabling failure detection.
- Sanitized the QuadTailsitter PX4 airframe comments and added report v61 plus
  an updated qtail25 final-polish test card.

### Notes

- No X-Plane ACF physics or bridge sensor-contract changes are included.
  qtail25 showed the 5 kg aircraft, body-axis pitot, TruthCapture timing, and
  front transition were credible; this is a PX4 airframe tuning slice.

---

## [3.4.48] - 2026-05-26

### Changed

- Retuned QuadTailsitter after qtail24 showed v3.4.47 fixed the large FW path
  and altitude error but still had a low-frequency yaw/roll wobble and an unsafe
  mission VTOL Land back-transition dive.
- Moved QuadTailsitter FW trim from `32` to `29 m/s`, softened NPFG/roll
  guidance, enlarged loiter radii to `1100 m`, and reduced FW yaw/roll
  differential-thrust scaling while preserving pitch authority.
- Shortened QuadTailsitter tailsitter back-transition duration from `6` to
  `4 s`, lowered the mission deceleration estimate, and added a small MC pitch
  authority increase so the aircraft can rotate toward hover sooner.
- Added report v60 and updated the QuadTailsitter test card with qtail24
  evidence, design sizing, and the v3.4.48 high-margin validation workflow.

### Notes

- No bridge sensor-contract or X-Plane ACF physics changes are included.
  qtail24 timing, body-axis pitot, mass, wing loading, and front transition were
  credible; the remaining failure was back-transition control/entry energy.

---

## [3.4.47] - 2026-05-26

### Changed

- Retuned QuadTailsitter after qtail23 showed that v3.4.46 loaded correctly but
  made FW guidance/back-transition too aggressive for the mission test.
- Raised QuadTailsitter autonomous MC climb rate from `1.5` to `2.0 m/s`.
- Moved QuadTailsitter FW trim from `24` to `32 m/s`, biased TECS toward
  altitude, softened FW roll-rate response, reduced FW differential-thrust
  scaling, and restored large `900 m` FW/RTL loiter radii.
- Shortened QuadTailsitter back-transition duration from `14 s` to `6 s` after
  source review showed PX4 tailsitters rotate by `90 deg / VT_B_TRANS_DUR`.
- Added report v59 and updated the QuadTailsitter test card for high-margin
  manual back-transition before any VTOL Land mission attempt.

### Notes

- No X-Plane aircraft physics or bridge sensor-contract changes are included.
  qtail23 evidence points to FW control/energy tuning, not a mass/wing/prop or
  pause/FPS sensor regression.

---

## [3.4.46] - 2026-05-26

### Changed

- Retuned QuadTailsitter fixed-wing energy and transition defaults after qtail22
  showed clean MC/front-transition behavior but excessive FW speed and unsafe
  back-transition entry energy.
- Raised QuadTailsitter autonomous climb rate modestly while preserving the
  qtail21/qtail22 lateral MC baseline.
- Reduced QuadTailsitter FW trim/max airspeed, front-transition throttle, and
  back-transition aggressiveness; tightened FW guidance from the first-transition
  guardrail values without enabling full VTOL Land mission testing yet.
- Hid the large rendered QuadTailsitter landing legs while preserving the fixed
  non-retractable contact gear physics.
- Added report v58 and updated the QuadTailsitter test card for the manual
  FW/back-transition gate.

### Notes

- No bridge sensor-contract changes are included. qtail22 TruthCapture timing
  and X-Plane diagnostics were clean, so this is a PX4 airframe/ACF visual
  slice.

---

## [3.4.45] - 2026-05-26

### Changed

- Promoted QuadTailsitter from MC-only recovery to the first straight-transition
  validation gate after qtail21 confirmed clean v3.4.44 parameter sync, bridge
  sensor behavior, and MC speed/attitude tracking.
- Updated the QuadTailsitter test card to preserve the qtail21 MC baseline,
  skip FW Orbit/RTL tuning until straight-transition evidence exists, and note
  that the large landing legs are intentionally fixed for ground-contact
  stability.
- Added report v57 with qtail21 evidence, acceptance metrics, and the v3.4.45
  first-transition workflow.

### Notes

- No QuadTailsitter PX4 params or ACF geometry changed in this release. The
  tiny remaining MC wobble is acceptable for transition gating and should not be
  over-tuned before FW evidence is collected.

---

## [3.4.44] - 2026-05-25

### Changed

- Reduced QuadTailsitter autonomous MC speed, acceleration, jerk, and tilt
  limits after qtail20 showed a `5 m/s` Go-To command producing a `14 m/s`
  actual speed burst and large pitch transient.
- Added a small QuadTailsitter pitch-rate damping increase while preserving the
  stable v3.4.43 hover/contact baseline.
- Updated QuadTailsitter documentation and report history for the qtail20
  evidence and clean-parameter retest workflow.

### Notes

- The qtail20 ULog did not load the v3.4.43 PX4 qtail acceleration/jerk
  defaults, so v3.4.44 requires a PX4 `make distclean` before judging the next
  run.

---

## [3.4.43] - 2026-05-25

### Fixed

- Kept barometer pressure live during the stationary-ground contract with tiny
  simulated sensor noise. This prevents PX4 from marking a mathematically frozen
  stationary pressure stream as `BARO #0 failed: STALE!` while keeping
  GPS/HIL_STATE position and velocity latched.
- Stabilized the QuadTailsitter landing-contact model by making the four gear
  supports fixed, increasing contact pad size, reducing strut travel, and
  increasing damping/support force.
- Lowered QuadTailsitter `LNDMC_Z_VEL_MAX` below landing crawl/speed thresholds
  to avoid PX4 startup correction noise.

### Changed

- Smoothed QuadTailsitter MC lateral commands with slightly lower horizontal
  acceleration and jerk defaults.
- Updated TruthCapture replay and tests to verify stationary baro liveness
  without moving latched GPS altitude.

---

## [3.4.42] - 2026-05-25

### Fixed

- Replaced the partial v3.4.41 stationary guards with a coherent
  stationary-ground sensor contract. While the aircraft is stationary on the
  ground and mapped motors are not commanding meaningful thrust, px4xplane now
  publishes attitude-consistent gravity, zero gyro rates, zero GPS/HIL_STATE
  velocity, and latched GPS/HIL_STATE/baro position/altitude.
- Suppressed GPS and barometer altitude noise only while the stationary-ground
  contract is active, preventing PX4 from seeing a climb/descent while the
  aircraft is sitting on its landing gear.
- Reset the stationary-ground contract on MAVLink reset so reconnects and
  aircraft changes cannot reuse a stale latched ground point.
- Used the guarded HIL_GPS position for magnetic-field update threshold checks.

### Added

- Added replay support for the stationary-ground contract in
  `tools/replay_truth_capture.py`.
- Added replay unit coverage for stationary contact acceleration/rate spikes.

---

## [3.4.41] - 2026-05-25

### Fixed

- Extended the stationary-ground guard from acceleration only to the full
  kinematic sensor contract. While X-Plane reports on-ground, very low AGL, and
  low horizontal speed, px4xplane now zeroes contact-model velocity before
  publishing HIL_GPS and HIL_STATE_QUATERNION velocity.
- Removed the vertical-speed threshold from the stationary-ground acceleration
  guard. The previous v3.4.40 threshold skipped the guard exactly when a
  landing-gear contact bounce produced a vertical-speed spike.
- Applied the same guarded local velocity to the body-axis pitot projection so
  tailsitter pitot simulation does not convert stationary gear bounce into
  false differential pressure.
- Applied the same guarded local velocity to GPS course-over-ground so a
  stationary contact bounce cannot update COG while GPS ground speed is guarded
  to zero.

### Added

- Added `ground_stationary_kinematics_guard_enabled` to `config.ini` and the
  config schema under the sensor-contract group.

---

## [3.4.40] - 2026-05-25

### Fixed

- Added a default-on ground-stationary accelerometer guard. If X-Plane reports
  gear/contact g-load spikes while the aircraft is stationary on the ground,
  px4xplane now sends stable gravity to PX4 instead of forwarding the contact
  impulse as HIL_SENSOR acceleration. This targets the immediate
  `vertical velocity unstable` regression seen before QuadTailsitter retesting.
- Reduced repeated `config.ini` path log spam by logging the resolved path only
  when it changes.

### Added

- Added `ground_stationary_accel_guard_enabled` to `config.ini` and the config
  schema under the sensor-contract group.

---

## [3.4.39] - 2026-05-24

### Fixed

- Fixed the timestamp provider's same-frame fallback so GPS/state/RC messages
  cannot jump to wall-clock elapsed time after an X-Plane pause or blocking
  dialog. Same-frame messages now advance by one microsecond relative to the
  previous HIL timestamp.
- Added connected pause/stall logging in the flight loop. The bridge now logs
  `[BRIDGE_PAUSE]` while X-Plane sim time is paused or stalled and
  `[BRIDGE_RESUME]` when sim time resumes, without resetting PX4 timestamps.

### Added

- Added `tools/check_px4_airframe_params.py` to compare ULog initial parameters
  against a PX4 airframe file. This catches stale SITL `parameters.bson` before
  flight behavior is used for tuning.
- Added `tools/analyze_qtailsitter_design.py` to summarize the QuadTailsitter
  ACF mass, battery, wing-loading estimate, prop RPM/tip Mach, hover thrust,
  and optional TruthCapture thrust/RPM/throttle evidence.
- Added offline tests for timestamp pause behavior and the new airframe tools.

### Changed

- Updated the QuadTailsitter test card and docs index for the v3.4.39 workflow.
  The next run must validate PX4 parameter sync before judging aircraft design
  or tuning.

---

## [3.4.38] - 2026-05-24

### Fixed

- Analyzed `/home/alireza/qtail15.zip`. The X-Plane side was v3.4.37 and the
  retargeted ACF loaded at `4.99 kg`, but the PX4 ULog still had stale
  parameters: `SYS_HAS_NUM_ASPD=1`, `MPC_THR_HOVER=0.27`,
  `VT_ARSP_TRANS=18`, and all `CA_ROTOR*_AX/AY=0`. This means qtail15 did not
  test the intended canted PX4 control allocation.
- Confirmed the user's MC tuning observation from the ULog. Changing only
  pitch-rate D and K did not remove the pitch oscillation; after
  `MC_PITCH_P` was reduced from `0.9` to `0.5`, pitch-rate RMS dropped from
  about `0.79 rad/s` to about `0.11 rad/s`, and the MC phase became usable.
- Confirmed the cockpit/QGC throttle display is not the X-Plane actuator path
  for this plugin path. TruthCapture showed cockpit throttle datarefs at zero
  while `throttle_used_ratio`, prop RPM, and PX4 `actuator_motors` were active.

### Changed

- Retuned the QuadTailsitter MC hover defaults for the measured 5 kg plant:
  `MC_PITCH_P=0.50`, `MC_PITCHRATE_K=0.85`, and `MPC_THR_HOVER=0.22`.
- Left FW orbit/TECS tuning mostly unchanged because qtail15 used stale PX4
  airspeed, thrust, and control-allocation defaults. The next FW conclusions
  must come from a run where the ULog shows the v3.4.38 sanity values.
- Updated the setup script warning so users choose `distclean` after pulling or
  copying new airframe parameter files.
- Updated the QuadTailsitter test card and added report v50.

---

## [3.4.37] - 2026-05-24

### Fixed

- Analyzed `/home/alireza/qtail14.zip`. The run used px4xplane v3.4.36,
  `Config Name: QuadTailsitter`, body-axis pitot `-Z`, and PX4 fork commit
  `ad499de`.
- Confirmed the recurring `Preflight Fail: Airspeed selector module down`
  warning was a commander prearm hardware-requirement race:
  `airspeed_selector` and simulator streaming warm up after commander starts,
  while `SYS_HAS_NUM_ASPD=1` required a fresh `airspeed_validated` message.
- Confirmed qtail14's uncontrolled climb was not stale params or missing pitot:
  the X-Plane aircraft still weighed `2.27 kg` (`5 lb`), so low/minimum motor
  outputs were already near hover thrust and the vehicle climbed far above the
  requested altitude.

### Changed

- Set X-Plane virtual-pitot airframes to avoid treating the SITL pitot as a
  prearm hardware requirement: `SYS_HAS_NUM_ASPD=0` for Cessna 172, TB2, and
  QuadTailsitter. Airspeed remains enabled for control where configured.
- Retargeted the QuadTailsitter ACF toward the requested 5 kg 6S design:
  empty mass `5 lb -> 11 lb`, max mass `6 lb -> 12.5 lb`, battery
  `64 V / 120 Wh / 60 A -> 22.2 V / 244 Wh / 180 A`, motor redline/max RPM
  `22,000 -> 25,000`, loaded prop design RPM `16,000 -> 17,000`, and prop
  design speed `40 kt -> 50 kt`.
- Added the first mirrored 5 degree motor-cant test in the ACF and PX4 rotor
  axes so the controller and simulated physics remain aligned.
- Rebased QuadTailsitter hover and transition defaults for the heavier model:
  `MPC_THR_HOVER=0.30`, `MPC_THR_MIN=0.08`, softer vertical/horizontal
  acceleration limits, `FW_AIRSPD_TRIM=28`, `FW_AIRSPD_MAX=40`,
  `VT_ARSP_TRANS=24`, `FW_THR_TRIM=0.16`, and `FW_THR_MAX=0.40`.
- Updated the QuadTailsitter test card and added report v49.

---

## [3.4.36] - 2026-05-23

### Fixed

- Analyzed `/home/alireza/qtail13.zip`. The archive is valid; TruthCapture
  recorded `42,479` frames over about `518 s` of sim time with zero dropped
  rows and about `82 Hz` mean callback rate.
- Confirmed v3.4.35 was active and PX4 used the modeled body-axis pitot
  (`airspeed_validated.airspeed_source=1`), so the FW path issue was not stale
  params or airspeedless transition.
- Identified the first quad-chute as a minimum fixed-wing altitude breach:
  transition was commanded with little margin above `VT_FW_MIN_ALT=40 m`, and
  PX4 triggered once local altitude dipped just below `40 m`.
- Confirmed FW orbit/RTL remained saturated because actual FW speed still ran
  about `35-42 m/s` while the vehicle was trying to capture the requested
  loiter/RTL path. The X-Plane aircraft prop model was still designed for
  `179 kt`, far above Quantix-class speed.

### Changed

- Retuned the packaged QuadTailsitter ACF prop model toward the Quantix-class
  envelope: prop design speed `179 kt -> 40 kt`, design RPM
  `28,916 -> 16,000`, and motor redline/max-power RPM `29,896 -> 22,000`.
- Retuned QuadTailsitter FW energy control:
  `FW_AIRSPD_TRIM=20`, `FW_AIRSPD_MAX=28`, `FW_THR_TRIM=0.05`,
  `FW_THR_MAX=0.25`, `FW_T_SPDWEIGHT=2.0`, `FW_T_RLL2THR=2.0`,
  `FW_PSP_OFF=4`, and `FW_R_LIM=28`.
- Widened and softened FW path following for the next test:
  `NAV_LOITER_RAD=900`, `RTL_LOITER_RAD=900`, `NPFG_PERIOD=34`,
  `NPFG_DAMPING=1.0`, and `NPFG_ROLL_TC=1.5`.
- Lowered the SITL fixed-wing minimum-altitude quad-chute gate to
  `VT_FW_MIN_ALT=30` while retaining transition altitude-loss protection.
- Updated the QuadTailsitter test card and added report v48.

---

## [3.4.35] - 2026-05-23

### Fixed

- Analyzed `/home/alireza/qtail12.zip`. The archive is valid; TruthCapture
  recorded `61,908` frames with zero dropped rows and about `80 Hz` mean
  callback rate.
- Confirmed PX4 used the modeled body-axis pitot: `airspeed_validated` stayed
  on sensor source `1`, and the tailsitter switched to fixed-wing at about
  `20 m/s` calibrated airspeed.
- Confirmed qtail12 FW orbit/RTL trouble was primarily lateral-guidance
  saturation at the current speed/radius, not a missing pitot or absent bank.
  FW speed ran about `32 m/s` in hold and `37 m/s` in RTL while NPFG repeatedly
  requested the `35 deg` roll-limit equivalent.
- Corrected body-axis pitot dynamic-pressure generation to use local X-Plane
  pressure/temperature density. Conventional `xplane_indicated` still uses the
  sea-level-equivalent IAS convention.

### Changed

- Reduced QuadTailsitter FW trim/throttle target for the next validation:
  `FW_AIRSPD_TRIM=22`, `FW_AIRSPD_MAX=34`, `FW_THR_TRIM=0.08`,
  `FW_THR_MIN=0.00`, and `FW_THR_MAX=0.35`.
- Lowered airspeed-gated transition thresholds to
  `VT_ARSP_BLEND=13` and `VT_ARSP_TRANS=18`.
- Increased QuadTailsitter FW path radius and softened NPFG:
  `NAV_LOITER_RAD=500`, `RTL_LOITER_RAD=500`, `NPFG_PERIOD=22`,
  `NPFG_DAMPING=0.85`, and `NPFG_ROLL_TC=1.2`.
- Updated the QuadTailsitter test card, custom-airframe config docs, and added
  report v47.

---

## [3.4.34] - 2026-05-23

### Fixed

- Analyzed `/home/alireza/qtail11.zip`. The log is valid and confirms the run
  still used `airspeedSource=xplane_indicated`, `FW_USE_AIRSPD=0`,
  `SYS_HAS_NUM_ASPD=0`, and `VT_ARSP_TRANS=0`; it did not use the modeled
  body-axis pitot for transition or TECS.
- Validated the QuadTailsitter body-axis pitot candidate from truth data:
  projecting velocity onto `pitotAxisBody=-Z` gives positive FW airspeed close
  to true airspeed, while native X-Plane IAS is negative in this attitude.
- Confirmed FW path error was mainly energy/radius mismatch, not absent bank:
  PX4 requested up to `+/-35 deg` FW roll and the transformed actual roll
  followed, but the aircraft flew around `42 m/s` against a `250 m` radius.
- Removed the startup land-detector correction by making
  `LNDMC_Z_VEL_MAX=0.30` valid with `MPC_LAND_CRWL=0.36`.

### Changed

- Switched QuadTailsitter to `airspeedSource=body_axis` with
  `pitotAxisBody=-Z`.
- Reused the configured airspeed source for `HIL_STATE_QUATERNION` airspeed
  fields so QGC and HIL_SENSOR use the same pitot basis.
- Made body-axis pitot projection subtract X-Plane's local wind vector before
  body-frame projection, with zero-wind fallback if those datarefs are absent.
- Enabled QuadTailsitter airspeed feedback:
  `FW_USE_AIRSPD=1`, `SYS_HAS_NUM_ASPD=1`, `VT_ARSP_BLEND=14`,
  and `VT_ARSP_TRANS=20`.
- Moved FW speed/energy to a first validated airspeed-controlled target:
  `FW_AIRSPD_MIN=16`, `FW_AIRSPD_TRIM=24`, `FW_AIRSPD_MAX=35`,
  `FW_THR_TRIM=0.12`, `FW_THR_MIN=0.03`, and `FW_THR_SLEW_MAX=0.30`.
- Softened back-transition and widened FW loiter:
  `VT_B_TRANS_DUR=8.0`, `VT_B_DEC_MSS=1.2`, `VT_B_TRANS_RAMP=6.0`,
  `NAV_LOITER_RAD=300`, and `RTL_LOITER_RAD=300`.
- Updated the QuadTailsitter test card and added report v46.

---

## [3.4.33] - 2026-05-22

### Fixed

- Analyzed `/home/alireza/qtail10.zip`. The log is valid and confirms
  QuadTailsitter reached fixed-wing state, but FW speed/path behavior was not
  acceptable.
- Confirmed the first transition failure was caused by starting below
  `VT_FW_MIN_ALT=40`, not by a bridge airspeed regression.
- Confirmed the second transition entered FW, then oversped around `40-60 m/s`
  while PX4's airspeedless FW controller assumed a synthetic `18 m/s` trim.
- Confirmed the commanded `120 m` FW loiter radius was too small for the
  qtail10 speed envelope.

### Changed

- Added reusable `airspeedSource` and `pitotAxisBody` config fields, plus
  schema/config-editor validation, for future pitot-axis experiments.
- Kept QuadTailsitter airspeedless for control, but documented the candidate
  physical pitot axis as `-Z` for this model.
- Reduced QuadTailsitter FW energy demand:
  `FW_THR_TRIM=0.20`, `FW_THR_MAX=0.45`, `FW_THR_SLEW_MAX=0.35`,
  and `FW_R_LIM=35`.
- Slowed the first transition candidate:
  `VT_F_TRANS_DUR=7.0`, `VT_F_TRANS_THR=0.45`,
  `VT_TRANS_MIN_TM=6.0`, `VT_F_TR_OL_TM=10.0`, and
  `VT_TRANS_TIMEOUT=25`.
- Increased FW loiter/RTL radii and altitude margins:
  `NAV_LOITER_RAD=250`, `RTL_LOITER_RAD=250`, `RTL_RETURN_ALT=80`,
  `RTL_DESCEND_ALT=60`, `NPFG_PERIOD=14`, and `NPFG_ROLL_TC=0.8`.
- Smoothed MC Go-To and landing defaults:
  `MPC_XY_P=0.18`, `MPC_ACC_HOR=1.8`, `MPC_JERK_AUTO=1.2`,
  `MPC_Z_V_AUTO_DN=1.4`, `MPC_LAND_SPEED=0.9`, and
  `MPC_LAND_CRWL=0.35`.
- Updated the QuadTailsitter test card and added report v45.

---

## [3.4.32] - 2026-05-22

### Fixed

- Analyzed `/home/alireza/qtail9.zip`. The long QuadTailsitter run is valid
  and showed six forward-transition attempts. None reached fixed-wing state.
- Confirmed the main transition blocker was airspeed gating: PX4 booted with
  `FW_USE_AIRSPD=1` and `VT_ARSP_TRANS=15`, while validated airspeed stayed
  around `0.8-2.0 m/s` even though truth groundspeed/true airspeed were high.
- Confirmed X-Plane indicated airspeed went negative during high-AoA tailsitter
  motion, while px4xplane v3.4.31 clamped negative IAS to zero before sending
  `HIL_SENSOR.diff_pressure`.
- Documented that qtail9 did not use a clean parameter load
  (`SYS_AUTOCONFIG=0`), so the next test must use `make distclean`, delete the
  PX4 SITL `parameters*.bson` files, or run `param reset_all` before judging
  the new defaults.

### Changed

- Preserve signed X-Plane IAS as signed `HIL_SENSOR.diff_pressure` using
  `q = 0.5 * rho * V * abs(V)`. This keeps reverse-flow/high-AoA evidence
  visible and matches PX4's signed differential-pressure topic.
- Updated `tools/replay_truth_capture.py` and its unit tests to match the
  signed differential-pressure sensor contract.
- Made QuadTailsitter transition airspeed policy internally consistent for the
  first transition tests: `FW_USE_AIRSPD=0`, `ASPD_DO_CHECKS=0`,
  `SYS_HAS_NUM_ASPD=0`, `VT_ARSP_BLEND=0`, and `VT_ARSP_TRANS=0`.
- Slowed and guarded the first transition candidate:
  `VT_F_TRANS_DUR=6.0`, `VT_F_TRANS_THR=0.65`,
  `VT_TRANS_TIMEOUT=20`, `VT_FW_MIN_ALT=40`, and
  `VT_QC_T_ALT_LOSS=35`.
- Promoted moderated live MC values from qtail9:
  `MPC_VEL_MANUAL=5.0`, `MPC_MAN_TILT_MAX=35`,
  `MPC_TILTMAX_AIR=40`, `MC_YAW_P=0.80`,
  `MC_YAWRATE_P=0.20`, `MC_YAWRATE_D=0.025`,
  `MC_YAWRATE_K=1.20`, and `MC_YAWRATE_MAX=65`.
- Updated the QuadTailsitter test card and added report v44.

---

## [3.4.31] - 2026-05-22

### Fixed

- Analyzed `/home/alireza/05_30_57.ulg`. The log is valid and captured the
  best QuadTailsitter multicopter-mode validation so far, using persistent live
  tuning values from qtail8/qtail9.
- Confirmed the reported Go-To brake/continue behavior is not a bridge,
  connection, GPS, or EKF fault. Most of it is expected PX4
  `DO_REPOSITION` behavior: Go-To is a point-hold command, so PX4 decelerates
  near the target before the next Go-To accelerates again. A secondary tracking
  issue remains: pitch lagged the trajectory setpoint when actual speed fell
  below demanded speed.
- Confirmed the qtail9 Orbit was stable with the larger radius/yaw policy:
  attitude errors stayed small and motor saturation was not the driver.

### Changed

- Promoted the successful higher-speed MC direction into the QuadTailsitter
  defaults while keeping acceleration/jerk guarded: `MPC_XY_CRUISE=5.0`,
  `MPC_XY_VEL_MAX=5.0`, `MPC_VEL_MANUAL=4.0`, `MPC_ACC_HOR=2.2`,
  `MPC_ACC_HOR_MAX=2.5`, `MPC_JERK_AUTO=1.5`, and `MPC_JERK_MAX=2.5`.
- Increased QuadTailsitter roll/pitch authority for smoother Go-To tracking:
  `MC_ROLL_P=0.9`, `MC_PITCH_P=0.9`, `MC_ROLLRATE_P=0.10`,
  `MC_PITCHRATE_P=0.10`, `MC_ROLLRATE_D=0.0008`,
  `MC_PITCHRATE_D=0.0008`, `MC_ROLLRATE_K=1.00`,
  `MC_PITCHRATE_K=1.00`, and rate limits of `80 deg/s`.
- Adopted a moderated version of the successful live yaw tune:
  `MC_YAW_P=0.75`, `MC_YAWRATE_P=0.16`, `MC_YAWRATE_D=0.015`, and
  `MC_YAWRATE_K=1.15`.
- Raised MC tilt authority to `32 deg`, below the qtail9 live `45 deg` value,
  and updated the test card for staged `3 -> 4 -> 5 m/s` Go-To validation.
- Left px4xplane bridge code, connection handling, aircraft geometry, motor
  cant, fixed-wing, and transition parameters unchanged in this slice.

---

## [3.4.30] - 2026-05-22

### Fixed

- Analyzed `/home/alireza/qtail8.zip`. The package was partly corrupt:
  `04_17_10.ulg` and TruthCapture `frames.csv` are not reliable, but
  `04_31_13.ulg` and TruthCapture metadata were usable.
- Confirmed the qtail8 crash happened after a `30 m` center-facing Orbit
  command while live-test values were already more aggressive than v3.4.29:
  `MPC_TILTMAX_AIR=45`, `MPC_XY_VEL_MAX=5`, and yaw D/K changes were active.
- Confirmed the usable qtail8 ULog does not show estimator loss before impact;
  the failure sequence was yaw/attitude/vertical coupling with motor thrust
  saturation during Orbit entry.

### Changed

- Raised QuadTailsitter MC-mode speed above the hover-recovery defaults without
  keeping the live crash envelope: `MPC_XY_CRUISE=3.0`,
  `MPC_XY_VEL_MAX=3.5`, `MPC_VEL_MANUAL=3.0`,
  `MPC_ACC_HOR=2.0`, `MPC_ACC_HOR_MAX=2.0`,
  `MPC_JERK_AUTO=1.2`, and `MPC_JERK_MAX=2.0`.
- Raised QuadTailsitter tilt authority to `25 deg`, not the live-test
  `45 deg`.
- Adopted moderate yaw-response improvements:
  `MC_YAW_P=0.55`, `MC_YAWRATE_P=0.10`, `MC_YAWRATE_I=0.015`,
  `MC_YAWRATE_D=0.005`, `MC_YAWRATE_K=1.10`, and `MC_YAWRATE_MAX=60`.
- Slowed automatic yaw setpoint motion to reduce Orbit entry yaw swings:
  `MPC_YAWRAUTO_MAX=35`, `MPC_YAWRAUTO_ACC=12`, and `MIS_YAW_ERR=30`.
- Moderately improved altitude hold: `MPC_Z_P=0.80`,
  `MPC_Z_VEL_P_ACC=2.5`, `MPC_Z_VEL_I_ACC=0.6`, and
  `MPC_Z_VEL_D_ACC=0.4`.
- Added report v42 and updated the QuadTailsitter test card for v3.4.30.

---

## [3.4.29] - 2026-05-22

### Fixed

- Analyzed `/home/alireza/qtail7.zip`. The v3.4.28 QuadTailsitter run
  completed takeoff, Go-To, Orbit, RTL, landing, and disarm without a crash,
  but the first Orbit capture still had large yaw and attitude lag.
- Confirmed the qtail7 issue is not an estimator or bridge sensor failure:
  TruthCapture timing was healthy, estimator innovation ratios stayed low, and
  PX4 control allocation reported torque achieved with motor headroom.
- Confirmed the QGC parameter-progress hang does not match the old v3.4.20
  px4xplane simulator-TCP regression; no connection code was changed in this
  slice.

### Changed

- Increased QuadTailsitter yaw tracking authority:
  `MC_YAWRATE_P=0.065`, `MC_YAWRATE_I=0.015`, and
  `MC_YAWRATE_MAX=60`.
- Changed QuadTailsitter `CA_ROTOR*_KM` from `+/-0.05` to `+/-0.04` so PX4
  commands more yaw motor differential for the same yaw torque while keeping
  the yaw row above the control allocator's weak-effectiveness cutoff.
- Raised roll/pitch rate P from `0.08` to `0.09` and softened horizontal
  trajectory demand: `MPC_XY_CRUISE=1.2`, `MPC_XY_VEL_MAX=1.8`,
  `MPC_ACC_HOR=0.7`, `MPC_ACC_HOR_MAX=1.0`, `MPC_JERK_AUTO=0.8`, and
  `MPC_JERK_MAX=1.5`.
- Explicitly set QuadTailsitter auto yaw defaults:
  `MPC_YAW_MODE=0`, `MPC_YAWRAUTO_MAX=60`, `MPC_YAWRAUTO_ACC=20`,
  `MIS_YAW_ERR=25`, and `MIS_YAW_TMT=5`.
- Added report v41 and updated the QuadTailsitter test card for v3.4.29.

---

## [3.4.28] - 2026-05-21

### Fixed

- Analyzed `/home/alireza/qtail6.zip`. The v3.4.27 QuadTailsitter run was the
  first controlled hover/Go-To/land/disarm cycle without a crash.
- Confirmed the qtail6 low-after-takeoff behavior was commanded by
  `MPC_TKO_SPEED=0.6` and `MPC_TKO_RAMP_T=2.5`, not a bridge sensor or motor
  mapping bug.
- Confirmed yaw allocation is now healthy: no sustained unallocated torque, no
  motor saturation, and clean estimator test ratios.

### Changed

- Raised QuadTailsitter takeoff climb/ramp authority to reduce the near-ground
  pause: `MPC_TKO_SPEED=1.0`, `MPC_TKO_RAMP_T=1.5`,
  `MPC_Z_V_AUTO_UP=1.0`, `MPC_Z_VEL_MAX_UP=1.5`, and
  `MPC_ACC_UP_MAX=2.2`.
- Modestly increased QuadTailsitter yaw tracking after qtail6 showed yaw lag
  with motor headroom: `MC_YAW_P=0.42`, `MC_YAW_WEIGHT=0.35`,
  `MC_YAWRATE_P=0.05`, `MC_YAWRATE_I=0.010`, and `MC_YAWRATE_MAX=45`.
- Documented the canted-motor decision: valid and likely useful, but deferred
  to a controlled ACF/control-allocation geometry slice if qtail7 still shows
  yaw lag.
- Added report v40 and updated the QuadTailsitter test card for v3.4.28.

---

## [3.4.27] - 2026-05-21

### Fixed

- Analyzed `/home/alireza/qtail5.zip`. The v3.4.26 yaw allocation fix was
  active and yaw torque was no longer persistently unallocated, but the aircraft
  still crashed from weak pitch/roll recovery.
- Changed QuadTailsitter `CA_ROTOR*_CT` from `6.5` to `2.0`. qtail5 showed the
  generic PX4 default overestimated this X-Plane model's control effectiveness,
  under-commanding motor differential while reporting torque achieved.
- Increased pitch/roll rate authority for hover recovery and reduced yaw demand
  for the lower `CT` value.

### Changed

- Reduced QuadTailsitter horizontal/tilt aggressiveness for the hover-only
  recovery test so position hold does not drive the aircraft into
  forward-flight-like excursions before attitude is stable.
- Set `MPC_THR_HOVER=0.27` and `MPC_TKO_RAMP_T=2.5` to reduce the near-ground
  pause observed in qtail5.
- Added report v39 and updated the QuadTailsitter test card for v3.4.27.

---

## [3.4.26] - 2026-05-21

### Fixed

- Analyzed `/home/alireza/qtail4.zip`. The crash was traced to PX4 control
  allocation, not TruthCapture timing or estimator data: PX4 requested yaw
  torque, but the QuadTailsitter motor outputs kept zero yaw differential and
  `control_allocator_status` reported yaw as unallocated.
- Restored QuadTailsitter rotor thrust coefficients to the PX4 allocator
  default `CA_ROTOR*_CT=6.5`. The previous `CT=1.0` with `KM=0.05` put yaw
  effectiveness exactly at the allocator weak-row cutoff, so yaw could be
  removed.
- Reduced yaw gains and yaw rate limit for the first restored-authority
  hover-only test.

### Changed

- Updated the QuadTailsitter hover test defaults to match the current 2.27 kg
  X-Plane aircraft: `MPC_THR_HOVER=0.25`, `MPC_TKO_RAMP_T=3`, and
  `MIS_TAKEOFF_ALT=1.5`.
- Added report v38 and updated the QuadTailsitter X-Plane 12 test card.

---

## [3.4.25] - 2026-05-20

### Fixed

- Analyzed `/home/alireza/qtail3.zip`. v3.4.24 removed the violent first-lift
  roll/pitch oscillation, but yaw torque was effectively unallocated after
  takeoff while motors were not saturated.
- Restored QuadTailsitter roll/pitch/yaw airmode (`MC_AIRMODE=2`) with the
  reduced qtail3 roll/pitch gains so PX4 can allocate yaw correction in hover.
- Set `MIS_TAKEOFF_ALT=3.0` for the hover-recovery test instead of inheriting
  the VTOL default `20 m` takeoff altitude.
- Set `LNDMC_Z_VEL_MAX=0.25` to match PX4's effective parameter clamp while
  keeping it below the configured landing crawl speed.

### Changed

- Moderately raised QuadTailsitter yaw response for the next hover-only test.
- Added report v37 and updated the QuadTailsitter test card for v3.4.25.

---

## [3.4.24] - 2026-05-20

### Fixed

- Analyzed `/home/alireza/qtail2.zip`. v3.4.23 used the corrected
  QuadTailsitter rotor geometry and healthy TruthCapture timing, but still
  diverged in hover after takeoff.
- Disabled the QuadTailsitter X-Plane aircraft's internal Artificial Stability
  engine mixing for motors 0-3 so px4xplane/PX4 has exclusive authority over
  motor throttle datarefs.
- Removed the QuadTailsitter land-detector startup correction by setting
  `LNDMC_Z_VEL_MAX=0.20`, below `MPC_LAND_CRWL=0.30`.

### Changed

- Added `aircraft/QuadTailsitter/` as source-controlled aircraft assets and
  copied it into CMake package outputs under `X-Plane_Aircraft/`.
- Reduced QuadTailsitter hover recovery gains, rate limits, yaw demand, takeoff
  speed, horizontal correction, acceleration, jerk, and tilt limits for the next
  controlled hover-only test.
- Added report v36 and updated the QuadTailsitter test card for v3.4.24.

---

## [3.4.23] - 2026-05-20

### Fixed

- Analyzed `/home/alireza/qtail1.zip`. The run used `SYS_AUTOSTART=5021`,
  px4xplane `v3.4.22`, and failed in multicopter hover before transition:
  takeoff at `29.37 s`, roll attitude failure at `33.22 s`.
- Corrected `5021_xplane_qtailsitter` rotor geometry. The previous file used
  X-Plane ACF positions directly; v3.4.23 converts X-Plane aircraft
  coordinates (`X=right`, `Y=up`, `Z=tail`) into PX4 body-frame meters.
- Reduced QuadTailsitter multicopter rate/attitude aggressiveness and rate
  limits for a hover-only recovery test.

### Changed

- Softened QuadTailsitter takeoff and disabled hover-thrust estimation for the
  first recovery run so initial climb behavior is deterministic.
- Renamed user-visible package, docs, and UI text to `QuadTailsitter`. The PX4
  target remains `xplane_qtailsitter` for build compatibility.
- Added report v35 and changed the QuadTailsitter test card to hover-only
  validation before transition testing resumes.

---

## [3.4.22] - 2026-05-20

### Fixed

- Adopted the accepted `evtol7` Alia back-transition ramp:
  `VT_B_TRANS_RAMP=15.0`. The ULog showed this was the only live
  back-transition parameter change in the accepted run.
- Reduced post-touchdown auto-disarm delay for Alia and Ehang SITL by setting
  `COM_DISARM_LAND=1.5`, while keeping the existing land-detector velocity
  headroom.
- Finalized Ehang orbit tuning from the successful live parameter sequence:
  `MC_ROLL_P=1.0`, `MC_PITCH_P=1.0`, `MC_ROLLRATE_K=3.0`, and
  `MC_PITCHRATE_K=2.05`.

### Changed

- Increased the visible PX4 SITL connection wait timeout from `30 s` to `60 s`
  so PX4's simulator TCP retry cadence has more room before the user has to
  retry manually.
- Reworked the QuadTailsitter PX4 airframe into a conservative first-test
  configuration based on PX4's official quadtailsitter SITL defaults, the
  uploaded X-Plane model geometry, and the validated px4xplane sensor settings.
- Added a QuadTailsitter X-Plane 12 first-test card and report v34.

---

## [3.4.21] - 2026-05-20

### Fixed

- Recovered the PX4 SITL connection path after v3.4.20 made the accepted
  simulator TCP stream non-blocking and could drop MAVLink packets on send
  back-pressure. v3.4.21 keeps the non-blocking listener/wait UI but restores
  the connected stream behavior from v3.4.19.
- Reduced false Alia startup airspeed noise by keeping `FW_USE_AIRSPD=1` and
  `ASPD_DO_CHECKS=1`, while no longer making the virtual SITL pitot a prearm
  hardware requirement through `SYS_HAS_NUM_ASPD`.
- Increased Alia and Ehang land-detector vertical headroom
  (`MPC_LAND_CRWL=0.30`, `LNDMC_Z_VEL_MAX=0.24`) after evtol6 showed
  post-touchdown `landed` state bouncing at about `0.5 m/s` estimated vertical
  velocity.
- Applied the ULog-tested Alia back-transition expected deceleration
  (`VT_B_DEC_MSS=1.5`) while leaving `VT_B_TRANS_DUR=35.0`.
- Fixed Ehang's lingering takeoff gate by raising `MPC_XY_ERR_MAX` to `10`,
  matching the Alia fix.

### Changed

- Moderately increased Ehang attitude response (`MC_ROLL_P=0.45`,
  `MC_PITCH_P=0.45`) and reduced horizontal velocity D damping
  (`MPC_XY_VEL_D_ACC=0.8`) without returning to the failed v3.4.18 high-gain
  values.
- Added report v33 with the v3.4.20 regression explanation and evtol6 Alia/Ehang
  log findings.

---

## [3.4.20] - 2026-05-20

### Fixed

- Tightened the PX4 SITL TCP connection lifecycle without changing Alia or
  Ehang flight-tuning parameters.
- While waiting for PX4, the menu now offers `Cancel SITL Connection Wait`
  instead of another ambiguous connect action.
- `XPluginStop` now closes both connected and waiting sockets, preventing a
  stale listener when X-Plane unloads the plugin during a connection wait.
- Accepted PX4 TCP sockets are explicitly configured non-blocking and
  `TCP_NODELAY`, so the plugin does not block X-Plane on a slow or stale peer.
- Send, receive, and select socket failures now disconnect cleanly and update
  the data-window last-event text instead of leaving the UI looking connected.

### Changed

- Removed `SO_REUSEPORT` from the listening socket setup. `SO_REUSEADDR` still
  allows quick reconnects, but duplicate listeners on port `4560` are no longer
  silently allowed.
- Connection wait timing now uses flight-loop wall time instead of simulator
  running-time datarefs, so the 30s timeout is user-visible wall-clock behavior.
- The Connection tab now shows the waiting state and the last connection event.
- Added report v32 documenting why occasional 5-10s waits can happen when PX4
  starts before the plugin and how this slice handles clean retry/cancel paths.

---

## [3.4.19] - 2026-05-19

### Fixed

- Raised Alia `MPC_XY_ERR_MAX` from `2` to `10` after `evtol5` showed PX4
  pausing the takeoff altitude trajectory while lateral residual stayed above
  the default multicopter threshold. No Alia fixed-wing, TECS, NPFG, transition,
  or prop-brake default was changed.
- Recovered Ehang 184 from the v3.4.18 roll oscillation by returning the
  multicopter attitude/position defaults to the last non-oscillatory baseline.
  The failed run showed actual roll reaching about `70 deg` against a capped
  `25 deg` setpoint with allocator saturation.
- Added the active airframe name to the connection HUD while waiting for PX4
  SITL, so wrong-airframe selection is visible before arming.

### Changed

- Menu and data-window display now show friendly airframe labels such as
  `Ehang 184` while preserving config keys like `ehang184` for compatibility.
- Added report v31 with the `evtol5` Alia/Ehang evidence and next validation
  checklist.

---

## [3.4.18] - 2026-05-19

### Fixed

- Disabled Alia lift-prop braking by default after `evtol4` showed the
  no-brake handoff recovered like the accepted v3.4.13 baseline until the brake
  window removed the remaining fixed-wing recovery margin.
- Restored Alia `VT_ARSP_TRANS=46.0` and `FW_PSP_OFF=3.0` while keeping the
  `ASPD_DO_CHECKS=1` virtual-pitot recovery from v3.4.17.
- Corrected Ehang 184 multicopter orbit tuning by raising attitude P gains from
  the under-responsive `0.3` values and reducing horizontal velocity D gain
  from the oscillation-prone `1.5` value.

### Changed

- Lowered Ehang horizontal cruise/manual/max speeds and tilt cap for smoother
  Orbit validation while keeping PX4-valid acceleration and jerk limits.
- Added report v30 with the `evtol4` Alia/Ehang evidence and next-test sanity
  checks.

---

## [3.4.17] - 2026-05-19

### Fixed

- Fixed the first-connect disconnect after changing aircraft or reloading a new
  flight by resetting `lastFlightTime` and bridge timing state before opening a
  new SITL server socket.
- Updated Alia `ASPD_DO_CHECKS=1` so the SITL virtual pitot keeps the
  missing-data check but avoids validator checks that can invalidate the sensor
  while the VTOL is still in low-airspeed multicopter flight.

### Changed

- Added report v29 with the `evtol3` findings: raw Alia airspeed was present,
  but `airspeed_validated` stopped after arming; Ehang was flying a long capture
  to a `146 m` commanded orbit, where only a very small visible roll angle is
  physically expected.

---

## [3.4.16] - 2026-05-19

### Fixed

- Split the Alia lift-prop brake mechanism into configurable modes:
  `feather`, `hard_lock`, and `prop_separate`. The default Alia retest path now
  uses the less invasive `feather` mode after the v3.4.15 hard-lock behavior
  correlated with unrecovered fixed-wing sink.
- Raised Alia transition margin to `VT_ARSP_TRANS=50.0` and `FW_PSP_OFF=4.0`
  so PX4 does not shut down lift support as close to the low-speed sink margin.
- Added `autoPropBrakeMode` to the config schema, validator, and static config
  editor so brake behavior is visible and validated instead of hidden in code.

### Changed

- Smoothed Ehang 184 orbit capture by reducing horizontal cruise/max/manual
  velocity and horizontal acceleration defaults.
- Updated Alia and Ehang X-Plane 12 test cards with v3.4.16 sanity checks,
  prop-brake policy expectations, and orbit-capture interpretation.

---

## [3.4.15] - 2026-05-19

### Fixed

- Recovered Alia fixed-wing loiter behavior after `evtol1`: restored the
  ULog-proven `FW_R_LIM=22` envelope because v3.4.14's `35 deg` bank limit
  caused sustained high-bank capture, throttle/elevator saturation, and
  altitude loss.
- Re-enabled Alia lift-prop braking through the generic opt-in brake policy
  with conservative command, dwell, and airspeed gates; the X-Plane failure
  seizure path remains disabled.
- Updated Alia and Ehang simulated accelerometer offset defaults from the latest
  clean logs so fresh `distclean` runs do not start from stale calibration.
- Set `LNDMC_Z_VEL_MAX=0.20` for Alia and Ehang so it is consistent with
  `MPC_LAND_CRWL=0.25` and does not trigger PX4's land-detector auto-correction.

### Changed

- Increased Alia fixed-wing loiter/RTL radii to `2000 m` for large-eVTOL
  validation turns.
- Tuned Ehang 184 first-log navigation defaults for smoother validation:
  `MPC_XY_CRUISE=6`, `MPC_XY_VEL_MAX=8`, `MPC_TKO_SPEED=2.0`,
  `MPC_VEL_MANUAL=6`, `MPC_Z_V_AUTO_DN=1.2`, and `NAV_ACC_RAD=12`.
- Updated Alia and Ehang test cards with the v3.4.15 sanity values and the QGC
  reposition/orbit caveat observed in the Ehang log.

---

## [3.4.14] - 2026-05-18

### Changed

- Polished Alia final-test parameters after the first broadly accepted v3.4.13
  run: faster high-altitude multicopter descent, slower final touchdown phase,
  larger fixed-wing loiter/RTL radii, and PX4-valid fixed-wing roll limit.
- Prepared Ehang 184 for controlled validation with PX4-valid low jerk,
  explicit acceleration limits, smoother yaw/tilt limits, slower takeoff, and
  a documented landing profile.
- Removed unsupported passenger/production certainty from Ehang comments and
  aligned sensor comments with the Alia X-Plane baro/FPS lessons.

### Fixed

- Moved tracked root reports into `docs/reports` and removed a stale tracked
  Windows console temp file from the repository.
- Added an Ehang 184 validation card to the packaged plugin docs.

---

## [3.4.13] - 2026-05-18

### Fixed

- Reduced normal X-Plane log noise by disabling compact bridge diagnostics by
  default and gating magnetic-field update messages behind verbose logging.
- Clarified that packaged `px4_airframes` are reference/install copies; PX4
  SITL reads the airframe file from the PX4 repository branch.

### Changed

- Recovered Alia fixed-wing loiter behavior after `alia-test7`: shortened NPFG
  look-ahead, lowered FW bank limit, reduced loiter radius to a passenger-level
  large-eVTOL turn, restored stronger roll-to-throttle compensation, and added
  a small fixed-wing pitch offset for front-transition/cruise trim.
- Set Alia `MPC_TKO_SPEED=3.0` so QGC takeoff climb matches the intended
  autonomous climb behavior.

---

## [3.4.12] - 2026-05-18

### Fixed

- Fixed the `No autoPropBrakes...` X-Plane log line so the disabled-brake and
  motor-brake summary messages no longer run together.
- Fixed stale package contents after config/docs/airframe-only edits by adding
  an always-run packaged-assets refresh target.

### Changed

- Tuned Alia fixed-wing mass/energy/path behavior from `alia-test6` evidence:
  matched `WEIGHT_BASE`/`WEIGHT_GROSS` to the X-Plane model mass, smoothed TECS
  altitude response, increased NPFG smoothing, slowed FW roll response, and
  changed back-transition deceleration to the measured value.
- Kept Alia prop braking disabled by default; the generic safer prop-brake
  policy remains available only for airframes that explicitly opt in.

---

## [3.4.11] - 2026-05-18

### Fixed

- Disabled Alia lift-prop auto braking by default after `alia-test5` proved the
  v3.4.10 brake policy could leave lift props feathered/stopped during
  transition recovery.
- Reworked optional prop-brake policy with all-motor apply gating, immediate
  all-motor release, dwell time, optional true-airspeed gate, and experimental
  failure-dataref use disabled by default.
- Released active prop brakes during actuator stale/reset/disconnect cleanup.

### Changed

- Narrowly adjusted Alia front-transition margin: `VT_ARSP_TRANS=46.0`,
  `VT_F_TRANS_DUR=45.0`, and `VT_F_TR_OL_TM=55.0`.
- Updated Alia simulated accel offset defaults to the latest no-warning
  test5-start calibration values.

---

## [3.4.10] - 2026-05-18

### Changed

- Reverted the unproven Alia v3.4.9 TECS, throttle, RTL descent, and
  back-transition tuning to the last better-tracking baseline.
- Raised X-Plane SITL barometer height noise to `1.0 m` across the X-Plane PX4
  airframes after ULog evidence showed `0.05 m` was still overconfident for
  multi-meter VTOL/FW baro innovations.
- Seeded Alia simulated accelerometer offsets so fresh `distclean` runs do not
  start from the zero-offset state that produced the high-accelerometer-bias
  warning in `alia-test4`.

### Fixed

- Strengthened optional prop brakes: configured lift props are now feathered,
  commanded to zero prop speed, and kept seized while braked instead of relying
  on the prop-separation failure trick alone.

---

## [3.4.9] - 2026-05-18

### Changed

- Added a ULog-guided Alia-250 tuning slice to reduce fixed-wing altitude
  phugoid behavior and back-transition climb/overshoot.
- Seeded PX4 simulated barometer device IDs in X-Plane airframe files so the
  intended disabled backup barometer priority applies on a fresh SITL rootfs.
- Relaxed X-Plane SITL barometer fusion noise from an overconfident 0.003 m to
  0.05 m.

### Fixed

- Added hysteresis to optional prop-brake activation so X-Plane failure datarefs
  do not chatter around exactly zero throttle.
- Clamped negative indicated airspeed before calculating HIL differential
  pressure.

---

## [3.4.8] - 2026-05-07

### Fixed

- Fixed Windows MSVC release-build portability by defining `NOMINMAX` and
  guarding a `std::numeric_limits<uint64_t>::max()` call from Windows `max`
  macro expansion.

### Unchanged

- Same config safety/editor/runtime behavior as v3.4.7.
- Alia PX4 parameters remain frozen at the v3.4.6 successful baseline.
- No bridge sensor sign/unit, GPS, HIL state, quaternion, actuator mapping, or
  prop-brake policy change is included in this release.

---

## [3.4.7] - 2026-05-07

### Added

- Added config safety validation foundation:
  - stale actuator mappings are cleared on reload
  - actuator outputs are finite-checked and clamped
  - invalid output ranges are skipped and logged
  - stale PX4 actuator input zeros configured actuator datarefs
  - inbound MAVLink receive drains a bounded multi-packet budget per frame
- Added `tools/validate_config.py` and tests for `config.ini` channel mappings,
  global field types/ranges, and prop-brake motor indices.
- Added runtime config validation status in the X-Plane UI.
- Added `Advanced` menu items for config validation, reload+validate, bridge
  diagnostics, docs location, and config editor location.
- Added `config/config_schema.json` as schema metadata for config fields,
  reload policy, supported channel types, and editor tooling.
- Added a static schema-backed config editor:
  - `docs/config-editor.html`
  - `docs/assets/config-editor.js`
  - packaged under `px4xplane/docs/`
- Added docs and reports for the config safety, validation UI, schema, and
  editor slices.

### Changed

- Moved bridge diagnostics from the normal menu into the Advanced menu.
- Packaged `config_schema.json` next to `config.ini`.
- Documented that `config.ini` remains the runtime format; JSON schema is
  metadata for validation and editor UX.

### Fixed

- Fixed several config/mapping hazards that could masquerade as tuning problems:
  stale channel mappings, non-finite actuator values, invalid ranges, stale PX4
  actuator streams, and missing float-array datarefs.

### Unchanged

- Alia PX4 parameters remain frozen at the v3.4.6 successful baseline.
- No bridge sensor sign/unit, GPS, HIL state, quaternion, or prop-brake policy
  change is included in this release.

---

## [3.4.6] - 2026-05-07

### Changed

- Restored the Alia fixed-wing/transition guidance defaults to the values
  observed in the full successful `alia-sitl1` ULog: TECS damping/time constants,
  NPFG period/damping, roll limit, and RTL/loiter radii.
- Updated the Alia test workflow to require a parameter-reset/distclean step
  when changing airframe defaults, because PX4 SITL can otherwise keep saved
  parameters across runs.

### Fixed

- Corrected the previous release assumption that the plugin-repo Alia file was
  the tested source of truth. The successful ULog and the PX4 fork state are now
  treated as the Alia baseline until controlled tuning proves otherwise.

---

## [3.4.5] - 2026-05-06

### Changed

- Cleaned the Alia airframe troubleshooting comments so the guidance matches the
  current TECS, NPFG, transition, and RTL/circle baseline values used for the
  next X-Plane 12 retest.

---

## [3.4.4] - 2026-05-06

### Fixed

- Fixed MSVC Windows release builds by protecting new diagnostics
  `std::min`/`std::max` calls from Windows `min`/`max` macros. This is a build
  portability hotfix; the validated v3.4.3 Alia sensor contract and tuning
  remain unchanged.

---

## [3.4.3] - 2026-05-06

### Added

#### Alia X-Plane 12 Validation Follow-Up
- Added bridge health diagnostics for effective sensor/GPS/state/RC rates,
  frame-period min/p50/p95/max, late/missed sends, timestamp monotonicity, and
  target-rate-vs-frame-rate warnings.
- Added `tools/replay_truth_capture.py` for deterministic core-contract replay
  from XPlaneTruthCapture folders or zip files.
- Added `tools/analyze_ulog_estimator.py` for ULog estimator-topic summaries and
  event-window analysis around RTL, barometer switches, landing, and disarm.
- Added replay unit tests for the current v3.4.3 sensor contract.
- Added `docs/ALIA_XPLANE12_TEST.md` as the next X-Plane 12 Alia run card.

### Changed

- Preserved the validated v3.4.3 sensor contract:
  `xacc=-g_axil`, `yacc=+g_side`, `zacc=-g_nrml`, GPS velocity from local
  velocity, true course-over-ground from local velocity, and HIL_STATE
  acceleration in milli-g.
- Removed outgoing HIL_SENSOR timestamp jitter so emitted MAVLink timestamps stay
  globally monotonic across messages in a frame.
- Reduced default debug log noise while keeping compact bridge diagnostics on.
- Cleaned `5020_xplane_alia250` comments and separated bridge-contract settings
  from aircraft-performance tuning settings.
- Made a conservative first Alia path/altitude tuning pass for TECS, NPFG, roll
  limit, loiter radius, and RTL loiter radius based on the successful May 6
  X-Plane 12 run.

### Fixed

- Fixed disconnect cleanup while waiting for PX4 SITL so the listening socket is
  closed when connection wait is cancelled or the plugin is disabled.

---

## [3.4.2] - 2025-02-01

### Added

#### FPS Warning HUD Indicator
- **Low FPS detection**: Shows subtle warning when X-Plane FPS drops below threshold
  - X-Plane frame rate directly affects sensor data quality sent to PX4
  - Low FPS causes reduced sensor update rate (EKF2 expects ~200Hz, gets X-Plane FPS)
  - Can lead to increased estimation noise and potential EKF2 warnings

- **Non-intrusive design**:
  - Small indicator in bottom-left corner (out of the way)
  - Color-coded severity: yellow for warning, orange-red for critical
  - Auto-hides when FPS recovers (5-second persistence prevents flickering)

- **Configurable via config.ini**:
  - `fps_warning_enabled`: Enable/disable the warning (default: true)
  - `fps_warning_threshold`: FPS below this triggers warning (default: 50)

- **Technical implementation**:
  - Rolling average FPS calculation (exponential moving average, α=0.1)
  - FPS sampled every 0.5 seconds for efficiency
  - Critical threshold at 60% of warning threshold

### Technical Details

#### Files Modified
- `include/ConnectionStatusHUD.h`: Added FPS monitoring state and methods
- `src/ConnectionStatusHUD.cpp`: FPS calculation and warning drawing
- `include/ConfigManager.h`: Added fps_warning_enabled, fps_warning_threshold
- `src/ConfigManager.cpp`: Load and validate FPS warning config
- `config/config.ini`: Added FPS warning configuration section
- `src/px4xplane.cpp`: Notify HUD when actively connected
- `include/VersionInfo.h`: VERSION 3.4.2, BUILD 009

---

## [3.1.1] - 2025-01-03

### Fixed

#### Critical Menu Handler Bug
- **Airframe selection not working**: Fixed airframe menu not switching configurations
  - Root cause: `refreshAirframesMenu()` was passing heap pointers instead of integer indices
  - Memory leak: `new std::string()` allocations never freed
  - Fixed: Changed to indexed for-loop passing `(void*)(intptr_t)i`
  - Also clicking airframes was incorrectly triggering "Show Data" window

#### Menu System Improvements
- **Added defensive constants**: `MENU_REF_MAIN` and `MENU_REF_AIRFRAMES` for clarity
- **Improved menu_handler()**: Explicit comparison instead of NULL check
- **Better error handling**: Added bounds checking and debug logging

#### PX4 SITL Reconnection Hang
- **Lockstep scheduler stuck after aircraft change**: Fixed reconnection hanging at "setting initial absolute time"
  - Root cause: Static timing variables (`lastSensorTime`, `lastGPSTime`, etc.) persisted across disconnect cycles
  - Timestamp jumps: Old timestamps (e.g., 1655s) mixed with new session (e.g., 5ms) confused lockstep scheduler
  - Solution: Added `g_needsTimingReset` flag to coordinate state cleanup
  - Reset logic in: `sendHILSensor()`, `sendHILGPS()`, `sendHILStateQuaternion()`, `computeAcceleration()`
  - Enhanced `MAVLinkManager::reset()` with comprehensive timing state cleanup

### Technical Details

#### Files Modified
- `src/px4xplane.cpp`: Menu handler fixes, defensive constants
- `src/MAVLinkManager.cpp`: Timing state reset logic, comprehensive documentation
- `include/VersionInfo.h`: VERSION 3.1.1, BUILD 002

---

## [3.1.0] - 2025-01-26

### Added

#### Professional 5-Tab UI System
- **Complete UI overhaul**: Replaced old overflow-prone single window with professional tabbed interface
- **5 organized tabs**:
  - **CONNECTION**: Status, timestamps, frame rate, connection details
  - **POSITION**: GPS coordinates, attitude (roll/pitch/yaw), angular velocities
  - **SENSORS**: IMU data, airspeed, environmental sensors
  - **CONTROLS**: Aircraft configuration, all 16 HIL actuator channels with live values
  - **MIXING**: Airframe configuration with real-time HIL data visualization
- **Per-tab scrolling**: Each tab scrolls independently - no more overflow
- **High-DPI support**: Professional rendering on all displays
- **Mouse wheel scrolling**: Smooth navigation through data

#### Performance Improvements
- **SITL rate increased to 400Hz**: HIL_SENSOR messages now sent at 400Hz
  - Better sensor fusion and state estimation
  - Reduced latency in control loop
  - Matches high-performance SITL standards

#### Documentation Overhaul
- **docs/DEVELOPER.md**: New streamlined developer guide
  - Quick reference workflow
  - Concise versioning guide (MAJOR.MINOR.PATCH)
  - CI/CD trigger reference
  - Common tasks and best practices
- **Consolidated documentation**: All docs moved to `docs/` folder
- **Removed redundancy**: Eliminated duplicate documentation

#### Configuration
- **Cleaned parameter files**: All 5 airframe parameter files reorganized
  - Removed obsolete parameters
  - Added clear documentation comments
  - Consistent formatting across all aircraft

### Fixed

#### Critical UI Bug
- **"Show Data" menu button**: Fixed handler not responding to clicks
  - Root cause: Incorrect `in_menu_ref` check
  - Now correctly identifies main menu vs submenu items
  - Added detailed debug logging

#### UI Integration
- **UIHandler not initialized**: Old drawing function was being used instead of new tabbed UI
  - Integrated `UIHandler::initialize()` in plugin startup
  - Changed to `UIHandler::drawMainWindow()`
  - Added mouse handlers for tab interaction and scrolling
  - Removed old drawing functions

### Changed

#### Menu Handler Improvements
- Better error handling with detailed debug output
- Clearer logic for menu item detection
- Improved airframe selection validation

### Technical Details

#### Files Modified
- `src/px4xplane.cpp`: Menu handler fix, UIHandler integration
- `config/config.ini`: HIL_SENSOR rate 250Hz → 400Hz
- `config/px4_params/*`: All 5 airframe files cleaned
- `include/VersionInfo.h`: VERSION 3.1.0, BUILD 001
- `CMakeLists.txt`: VERSION 3.1.0
- `docs/DEVELOPER.md`: New streamlined guide
- `README.md`: Updated workflow documentation link
- Moved: `BUILD_SYSTEM.md` → `docs/BUILD_SYSTEM.md`
- Removed: `VERSIONING.md`, `docs/GITHUB_ACTIONS.md`

---

## [3.0.1] - 2025-01-26

### Added

#### Documentation
- **VERSIONING.md**: Comprehensive developer guide for versioning and releases
  - Step-by-step release process with semantic versioning
  - Branch strategy (develop/master/tags)
  - Common scenarios (patch/minor/major releases)
  - GitHub Actions CI/CD reference with workflow triggers
  - Best practices and troubleshooting guide

#### README Improvements
- Added v3.0 video tutorial placeholder
- Highlighted major v3.0 improvements:
  - EKF2 stability fixes (altitude drift resolution)
  - Multi-threaded communication architecture
  - Improved state estimators (GPS/Baro/IMU fusion)
- Added PX4 official integration status section
- Reference to automated setup script for easy installation

### Fixed

#### Release Workflow
- **Windows Release Upload**: Enhanced build output verification
  - Added verification step to check build structure
  - Explicit plugin binary existence validation
  - Detailed error messages for debugging
  - Better error handling for missing directories
- Improved diagnostics for troubleshooting failed releases

### Technical Details

#### Files Modified
- `VERSIONING.md`: New comprehensive versioning guide
- `README.md`: Enhanced v3.0 feature highlights, PX4 integration status
- `.github/workflows/release.yml`: Build verification and error handling
- `include/VersionInfo.h`: VERSION 3.0.1, BUILD 002
- `CMakeLists.txt`: VERSION 3.0.1

---

## [3.0.0] - 2025-01-26

### 🎉 Major Release - Breaking Changes

This is a major version release with significant structural changes to comply with X-Plane SDK standards and implement professional CI/CD workflows.

### ⚠️ BREAKING CHANGES

#### Plugin Structure Reorganization
- **config.ini moved to 64/ folder**: Now located WITH the binary instead of parent folder
- **PX4 parameters organized**: Moved to dedicated `px4_airframes/` subdirectory
- **Installation method changed**: Must copy entire `px4xplane/` folder (not individual files)

**Migration Required**: Remove old v2.x installation and reinstall with new structure.

**Old Structure (v2.x)**:
```
px4xplane/
├── config.ini                    ← Root folder
├── 64/win.xpl
└── 5001_xplane_cessna172         ← Scattered params
```

**New Structure (v3.0.0)**:
```
px4xplane/
├── 64/
│   ├── win.xpl
│   └── config.ini                ← WITH binary
├── px4_airframes/                ← Organized
│   ├── 5001_xplane_cessna172
│   ├── 5002_xplane_tb2
│   ├── 5010_xplane_ehang184
│   ├── 5020_xplane_alia250
│   └── 5021_xplane_qtailsitter
└── README.md
```

### Added

#### Automated CI/CD System
- **GitHub Actions workflows**: Automatic builds on every push to master
- **Multi-platform builds**: Windows, Linux, macOS built in parallel (5-15 minutes)
- **Automated releases**: Push version tag to create release with pre-built binaries
- **Artifact retention**: 90-day storage for testing builds
- **Status badges**: Real-time build status on README
- **develop branch strategy**: No builds on develop (saves CI minutes), only master

#### Cross-Platform Build System
- **Unified CMake**: Single build system for all platforms (Windows/Linux/macOS)
- **Native Makefiles**: `Makefile.linux` and `Makefile.macos` for direct builds
- **Build consistency**: All methods (CMake, Visual Studio, Make) produce identical structure
- **Universal Binary (macOS)**: Single binary supports Intel x86_64 + Apple Silicon ARM64

#### Documentation
- **CHANGELOG.md**: Comprehensive version history (this file)
- **docs/GITHUB_ACTIONS.md**: Complete CI/CD workflow guide for developers
- **docs/BUILD_SYSTEM.md**: Technical build system documentation
- **Enhanced README.md**: Status badges, simplified installation, release workflow

### Fixed

#### Critical Build Failures
- **Linux case-sensitive includes**: Fixed `ConfigReader.h` → `configReader.h`
- **macOS missing headers**: Added `#include <cstring>` for `strlen()` in UIConstants.h
- **macOS linker errors**: Removed incorrect XPLM/XPWidgets framework linking
- **macOS deprecated warnings**: Updated `-undefined suppress` → `-undefined dynamic_lookup`
- **Windows output path**: Fixed MSVC multi-config directory nesting issue

#### Code Quality
- **Uninitialized variables**: Zero-initialized RC input struct (`hil_rc_inputs = {}`)
- **Compiler warnings**: Fixed unused variable warnings across all platforms

### Changed

#### macOS Plugin Linking
- **Standard X-Plane approach**: Using flat namespace with dynamic lookup
- **Framework cleanup**: Removed non-existent XPLM/XPWidgets frameworks
- **Runtime symbol resolution**: X-Plane provides XPLM symbols at load time
- **OpenGL only**: Only link OpenGL framework (required for rendering)

#### Visual Studio Project
- **Debug build fixed**: config.ini now copies to 64/ folder (was parent folder)
- **px4_airframes/ added**: Both Debug and Release now copy organized param files
- **Consistency achieved**: Debug and Release produce identical structures
- **Post-build messages**: Enhanced progress output with clear file locations

#### CMakeLists.txt
- **Version**: Updated project(px4xplane VERSION 3.0.0)
- **File organization**: config.ini → 64/, params → px4_airframes/
- **Post-build automation**: Directory creation and file copying
- **Windows MSVC fix**: Per-configuration output directories

### Removed

#### Dependencies
- **GeographicLib submodule**: Removed (not used in project code)

### Technical Details

#### Git Submodules
- **Eigen**: Added as proper submodule (linear algebra library)
- **Workflow configuration**: `actions/checkout@v4` with `submodules: recursive`

#### Files Modified
- `CMakeLists.txt`: Version 3.0.0, file organization, Windows MSVC fix
- `px4-xplane.vcxproj`: Debug/Release consistency, px4_airframes/ support
- `.github/workflows/build.yml`: Build only on master branch
- `.github/workflows/release.yml`: Automated release creation
- `include/VersionInfo.h`: VERSION 3.0.0, BUILD 001
- `src/ConnectionStatusHUD.cpp`: Added `<cstring>` header
- `src/px4xplane.cpp`: Fixed case-sensitive include
- `src/MAVLinkManager.cpp`: Zero-initialized RC inputs

### Migration Guide

#### Upgrading from v2.x to v3.0.0

1. **Remove old installation**:
   ```bash
   rm -rf "X-Plane 12/Resources/plugins/px4xplane"
   ```

2. **Download v3.0.0**:
   - Visit: https://github.com/alireza787b/px4xplane/releases/tag/v3.0.0
   - Download ZIP for your platform

3. **Install new version**:
   ```bash
   unzip px4xplane-{platform}-v3.0.0.zip
   cp -r px4xplane/ "X-Plane 12/Resources/plugins/"
   ```

4. **Verify**:
   - Launch X-Plane
   - Plugins → Plugin Admin → px4xplane should appear
   - Check PX4-SITL menu is available

5. **Config location changed**:
   - Old: `px4xplane/config.ini`
   - New: `px4xplane/64/config.ini`
   - Edit new location if custom configuration needed

---

## [2.5.1] - 2025-01-19

### Fixed
- **Height Estimate Stability**: Resolved altitude drift and EKF2 reset issues affecting flight stability
  - Reduced GPS altitude noise from σ=0.5m to σ=0.15m (matches high-quality GPS vertical accuracy)
  - GPS altitude now varies smoothly ±30cm instead of jumping ±1m
  - EKF2 properly prioritizes barometer (±3mm) over GPS for altitude estimation
  - Eliminated false climb/descent detections caused by GPS noise
  - Height estimate resets reduced from ~10/hour to rare occurrences

- **EKF2 Parameter Consistency**: Updated EKF2_GPS_P_NOISE across all aircraft configurations
  - Changed from 0.01 to 0.15 in: Alia250, eHang184, Cessna172, TB2, QTailsitter
  - Added comprehensive documentation explaining GPS-barometer sensor fusion
  - Parameter now accurately reflects GPS vertical accuracy (15cm)

- **PX4 Parameter Compatibility**: Removed non-existent parameters causing startup errors
  - Removed VT_F_TRANS_RAMP (VTOL transition ramp - version-specific)
  - Removed VT_DWN_PITCH_MAX (VTOL pitch limit - version-specific)
  - Removed FW_T_SPD_OMEGA (TECS speed filter - version-specific)
  - Removed FW_T_I_GAIN_THR (TECS throttle integrator - version-specific)
  - Added explanatory comments for each removed parameter

### Added
- **Connection Status HUD**: Professional on-screen feedback for SITL connection attempts
  - Real-time progress indicator with elapsed time (0-30 seconds)
  - Visual connection states: WAITING (yellow), CONNECTED (green), TIMEOUT (red), ERROR (orange)
  - Auto-fade success notification after 3 seconds
  - 30-second timeout with actionable error messages
  - Top-center positioning following X-Plane UI standards
  - Minimal, informative design with main text + subtitle format
  - Platform-specific OpenGL rendering (Windows/Mac/Linux)
  - Configurable via `show_connection_status_hud` in config.ini (enabled by default)

- **Configuration Options**:
  - `show_connection_status_hud`: Enable/disable connection HUD overlay
  - `debug_log_ekf_innovations`: Log EKF2 innovation data for diagnostics

### Changed
- **Performance Optimization**: Debug logging disabled by default for production use
  - `debug_verbose_logging`: false (was true)
  - `debug_log_sensor_timing`: false (was true)
  - `debug_log_sensor_values`: false (was true)
  - Reduces X-Plane Log.txt file size and improves performance
  - Can be re-enabled for development/tuning via config.ini

- **GPS Sensor Model**: Improved realism and EKF2 compatibility
  - GPS altitude noise: σ=0.5m → σ=0.15m (3.3× improvement)
  - Matches u-blox F9P high-quality GPS vertical accuracy specifications
  - 95% of GPS readings now within ±30cm (was ±1m)
  - Better alignment between sensor noise and EKF2 parameters

### Technical Details

#### Height Stability Root Cause Analysis
The height estimate drift issue was caused by a mismatch between GPS sensor noise and EKF2 parameters:

**Problem:**
- GPS altitude noise: σ=0.5m → random jumps of ±1m
- EKF2_GPS_P_NOISE: 0.01 → EKF2 trusted GPS as having 1cm accuracy
- EKF2 interpreted ±1m GPS jumps as actual altitude changes
- Result: False climb/descent detection → height estimate reset

**Solution:**
- GPS altitude noise: σ=0.15m → smooth variation of ±30cm
- EKF2_GPS_P_NOISE: 0.15 → EKF2 expects 15cm GPS uncertainty
- EKF2 now correctly weights barometer (±3mm) > GPS (±30cm)
- Result: Stable height estimate with rare resets

#### Connection HUD Implementation
- Uses X-Plane SDK `XPLMRegisterDrawCallback()` for HUD rendering
- Non-blocking draw callback executed every frame during `xplm_Phase_Window`
- Platform-specific OpenGL includes for Windows/Mac/Linux compatibility
- Static class design with no dynamic memory allocation
- Enum class `Status` with scoped values to avoid Windows `ERROR` macro conflict
- Text centering using approximate character width calculation
- Background opacity: 85% for optimal contrast while maintaining transparency

#### Files Modified
- `src/MAVLinkManager.cpp`: GPS altitude noise model (line 43)
- `config/px4_params/5010_xplane_ehang184`: EKF2_GPS_P_NOISE (line 350)
- `config/px4_params/5001_xplane_cessna172`: EKF2_GPS_P_NOISE (line 230)
- `config/px4_params/5002_xplane_tb2`: EKF2_GPS_P_NOISE (line 286)
- `config/px4_params/5021_xplane_qtailsitter`: EKF2_GPS_P_NOISE (line 174)
- `config/px4_params/5020_xplane_alia250`: EKF2_GPS_P_NOISE (line 297), removed invalid params
- `src/ConnectionStatusHUD.cpp`: New HUD implementation
- `include/ConnectionStatusHUD.h`: New HUD header
- `src/px4xplane.cpp`: HUD integration (lines 31, 205-206, 553-592, 722)
- `include/ConfigManager.h`: New config flags (lines 109, 112)
- `src/ConfigManager.cpp`: Config loading (lines 123-182)
- `config/config.ini`: Updated configuration (lines 39-48)
- `px4-xplane.vcxproj`: Added ConnectionStatusHUD to build

---

## [2.5.0] - 2025-01-15

### Fixed
- **Critical Sensor Stability**: Resolved BARO STALE errors and vertical velocity oscillation
  - Increased HIL_SENSOR update rate from 100Hz to 250Hz
  - Enhanced barometer noise modeling (3cm std dev)
  - Synchronized GPS-Barometer altitude quantization (5mm resolution)
  - Relaxed EKF2 innovation gates (BARO_GATE, GPS_V_GATE: 8.0)
  - Tuned COM_ARM_EKF_VEL parameter (0.8)

### Added
- Enhanced User Interface with professional high-contrast design
- Cross-platform support for Linux and macOS (native makefiles)
- Portability improvements with relative paths

### Changed
- Project structure improvements for better maintainability
- External dependencies properly excluded from version control

---

## [2.0.0] - 2024-12-26

### Added
- Multi-airframe support (Ehang 184, Alia 250, Cessna 172, Bayraktar TB2)
- In-menu airframe selection
- Automated setup script for WSL/Linux/macOS
- Detailed configuration instructions in config.ini
- Pre-loaded PX4 parameters

### Fixed
- Multiple sensor problems and inconsistencies

---

## [1.0.0] - 2024-11-15

### Added
- Initial release of PX4-XPlane plugin
- Basic MAVLink HIL integration
- Support for quadcopter airframes
- Configuration file system
