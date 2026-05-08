# px4xplane Journey Log

This log preserves project decisions, evidence, and next actions across the longer px4xplane recovery effort.

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
