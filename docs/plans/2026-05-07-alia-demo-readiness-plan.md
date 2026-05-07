# Alia Demo Readiness Plan

Date: 2026-05-07

This document is a planning checkpoint, not an implementation change. It answers
the retest questions, records what is proven, and proposes the next phase slices
before we change more flight behavior.

## Immediate Answer

The run that "barely flew" in the uploaded `alia-sitl2` evidence was not proven
to be Alia:

- `/home/alireza/alia-sitl2/19_07_20.ulg` has `SYS_AUTOSTART=5010`, which is
  the Ehang airframe, not Alia. Alia is `SYS_AUTOSTART=5020`.
- `/home/alireza/alia-sitl2/19_10_42.ulg` is Alia, but it is only about
  36-38 seconds of data and does not cover the full forward transition, RTL,
  back-transition, landing, and disarm workflow.
- The full successful Alia run remains
  `/home/alireza/alia-sitl1/05_17_33.ulg`.

So the correct conclusion is narrower than "the new tuning was bad":

- The bad uploaded ULog is not valid Alia tuning evidence.
- The short Alia ULog does not prove a full-flight tuning regression.
- The unproven v3.4.5 follow-up tuning was still a process mistake because it
  was not backed by a controlled retest.
- v3.4.6 intentionally restores the successful Alia ULog baseline.

## What To Test Now

For the next lab test, use the v3.4.6 package I gave you:

- `/home/alireza/px4xplane-windows-v3.4.6-test014-with-px4-params.zip`
- `/home/alireza/XPlaneTruthCapture-Windows-v0.1.7.zip`
- `/home/alireza/5020_xplane_alia250_v3.4.6_test014`

Reason: this is the known successful Alia baseline plus the fixed
TruthCapture package. It is the right baseline before we tune further.

Before flying:

1. Replace the Alia airframe file in the PX4 fork.
2. Run the setup script with `distclean` once, or otherwise reset SITL
   parameters. Manual file copy only changes the file on disk; PX4 can still
   load saved `parameters.bson`.
3. Confirm PX4 startup says `SYS_AUTOSTART=5020`.
4. Confirm XPlaneTruthCapture appears in the X-Plane plugin menu and shows
   recording state when capture starts.

## Build And Package Clarification

The TruthCapture issue was not "local builds always fail" and not "only GitHub
Actions works".

The actual issue was:

- The local v0.1.6 Windows zip was built with MinGW.
- Its `win.xpl` depended on `libgcc_s_seh-1.dll` and `libstdc++-6.dll`.
- Those DLLs were not in the zip, so X-Plane could fail to load it before the
  menu appeared.

What v0.1.7 fixes:

- Local MinGW TruthCapture builds now statically link the GCC runtime.
- The local v0.1.7 `win.xpl` imports only `XPLM_64.dll`, `KERNEL32.dll`, and
  `msvcrt.dll`.

Remaining packaging policy:

- GitHub Actions Windows assets are MSVC builds and can import VC runtime DLLs.
  That is normal, but for lab tests we should prefer the explicitly named
  self-contained Windows test zip when available.
- Next build-system slice should make Windows packaging policy explicit:
  either static MSVC runtime, bundled runtime, or a CI dependency gate that
  blocks unsafe release assets.

## Alia Realism Target

The goal is not just "it flies". The target is a polished passenger-grade demo:

- calm vertical takeoff and hover
- smooth front transition with no abrupt pitch, sink, or throttle step
- fixed-wing cruise at plausible Alia speeds and attitudes
- smooth RTL/circle tracking with controlled bank angle and low altitude error
- no estimator warnings in flight
- no GPS rejection bursts
- no noisy user-facing messages
- clean back-transition, descent, landing, disarm, and log close
- repeatability across reasonable FPS and dynamic frame-rate changes

Public references constrain, but do not fully define, exact simulator tuning:

- BETA publicly describes ALIA VTOL and CTOL variants, all-electric operation,
  336 nm demonstrated range, 200 cubic feet cargo capacity, less than one hour
  charge time, all-weather IFR operation, and 5+1 passenger plus pilot capacity.
- BETA's 2026 annual report describes ALIA VTOL A250 as the VTOL aircraft and
  says the defense VTOL MV250 draws heavily from the ALIA platform; it also
  mentions customer/military specifications around carrying approximately
  2,000 lb for 250 nautical miles for the defense variant.
- FAA registry entries confirm experimental ALIA-250/A250 electric aircraft
  records but do not give a type certificate data sheet with certified limits.

Therefore we should not invent "real certified Alia" performance numbers. We
should tune the X-Plane Alia to believable, smooth, internally consistent
behavior and clearly document what is model-derived, log-derived, and public
source-derived.

Sources:

- BETA public site: https://beta.team/
- BETA 2026 10-K: https://investors.beta.team/financials/annual-reports/content/0001628280-26-015838/bta-20251231.htm
- FAA registry example ALIA-250: https://registry.faa.gov/AircraftInquiry/Search/NNumberResult?nNumberTxt=250UT
- FAA registry example A250: https://registry.faa.gov/aircraftinquiry/Search/NNumberResult?NNumberTxt=250XT

## PX4 Tuning Order

PX4's own guidance supports a staged tuning order:

1. Multicopter rate and attitude loops.
2. Multicopter position and vertical response.
3. Fixed-wing rate and attitude loops.
4. Fixed-wing trim, airspeed, throttle, and pitch limits.
5. TECS altitude/airspeed behavior.
6. NPFG path tracking.
7. VTOL front transition.
8. VTOL back-transition and landing.

Important PX4 guidance:

- PX4 fixed-wing position tuning states that TECS controls altitude/airspeed and
  NPFG controls horizontal path following.
- PX4 fixed-wing tuning says the attitude controller should be well tuned before
  TECS tuning.
- PX4 multicopter tuning says the rate controller is the inner-most loop and
  poor rate tuning shows up as twitches/oscillation in position mode.
- PX4 VTOL QuadPlane docs say MC and FW modes must be tuned before transition
  tuning, and that transition airspeed/throttle/duration must be chosen to avoid
  stall or unstable transition.

Sources:

- PX4 FW TECS/NPFG tuning: https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 MC PID tuning: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 VTOL QuadPlane tuning: https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration

## Tuning Plan

### Slice A - Baseline Retest

Run v3.4.6 with TruthCapture v0.1.7 and a PX4 parameter reset. Do not tune yet.

Artifacts required:

- PX4 ULog
- PX4 CLI log
- X-Plane `Log.txt`
- XPlaneTruthCapture run zip/folder

Acceptance:

- `SYS_AUTOSTART=5020`
- full workflow completes
- no in-flight EKF warnings
- TruthCapture records successfully
- bridge diagnostics show actual frame-rate-limited message rates

### Slice B - Evidence Extraction

Build one report from ULog plus truth capture:

- hover attitude/rate tracking
- vertical velocity and altitude tracking during takeoff/landing
- front transition pitch, sink, airspeed, and throttle
- fixed-wing cruise speed, pitch, throttle, bank, climb/sink
- RTL/circle radius, cross-track/path error, altitude error
- back-transition deceleration, sink, attitude, throttle handover
- EKF innovation/test-ratio windows for GPS, baro height, and GNSS velocity
- baro source switch timing and effect

No parameter changes until this report identifies the first limiting loop.

### Slice C - Alia Param Cleanup

Split the airframe file into clearly labeled sections:

- bridge-required SITL params
- estimator/sensor params
- multicopter control
- fixed-wing control
- TECS/NPFG guidance
- VTOL transition
- RTL/mission/acceptance radii
- logging/test-only params

Remove certainty language. Each non-obvious value should have one of:

- PX4 default
- X-Plane model requirement
- ULog-proven baseline
- candidate tuning value, with test ID

### Slice D - Passenger-Grade Tuning

Tune one behavior at a time:

- Hover: reduce vertical bobbing and attitude twitching before touching FW.
- Front transition: smooth pusher throttle ramp, pitch, sink, and transition
  speed.
- Fixed-wing cruise: set trim airspeed/throttle/pitch from the X-Plane model.
- TECS: reduce altitude oscillation without hiding bad trim.
- NPFG: tune path following and bank behavior from actual RTL/circle traces.
- Back-transition: smooth deceleration and height control.
- Landing: remove shutdown-only warning if possible, or prove it is harmless.

The demo target is smoothness and credibility, not aggressive tracking.

## X-Plane Side Config And Mixing Findings

The X-Plane-side config deserves its own implementation slice before we trust
more airframes.

High-risk findings:

- `ConfigManager::parseConfig()` does not clear old `actuatorConfigs`, so
  switching from a high-channel airframe to a low-channel airframe can leave
  stale channels driven.
- Config is not loaded at plugin startup, even though UI text says it is.
  Config is loaded on connection or manual reload, so UI can show no/stale
  mapping before that.
- Docs mention unsupported data types like `int`/`intArray`; parser only
  handles `float` and `floatArray`.
- Missing datarefs are mostly silent during writes.
- Float-array values in the mixing UI are displayed as scalar `getFloat()`,
  not by configured array index.
- Actuator commands are scaled but not clamped or NaN-checked before writes.
- Actuator freshness is undefined: if PX4 stops sending new
  `HIL_ACTUATOR_CONTROLS`, the plugin keeps writing the last values.
- Inbound MAVLink reads only one 255-byte chunk per frame, which can backlog at
  low FPS or during bursts.
- Prop brake logic runs too often, uses exact `throttle == 0.0`, and simulates
  brakes through engine failure/prop-separation datarefs. That is brittle.
- Throttle ranges are ambiguous: comments say motors use 0..1, but several
  mappings use -1..1.
- TB2 mapping may not match its PX4 airframe outputs and should be verified
  before claiming multi-airframe quality.

Implementation order:

1. Clear actuator config on reload/switch.
2. Load and validate config at plugin startup.
3. Add a typed validation pass: dataref exists, type is supported, array index
   is valid, range is finite, min/max are sane.
4. Clamp and NaN-check actuator outputs.
5. Add actuator freshness timeout and failsafe zeroing.
6. Drain inbound MAVLink packets per frame up to a bounded budget.
7. Move prop brake behavior behind an explicit typed policy.
8. Add tests for config parsing, stale-channel cleanup, scaling/clamping,
   missing datarefs, array-index display, and actuator timeout.

## Config Editor Recommendation

Yes, a local UI/editor is worth building, but only after the schema exists.
Do not build a raw INI text editor.

Recommended architecture:

- Keep `config.ini` supported for backward compatibility.
- Introduce a typed schema as the source of truth, likely JSON or YAML.
- Generate `config.ini` from the typed schema until the plugin reads the schema
  directly.
- Build a local static HTML/JS editor in the repo, served by file open or a
  tiny local static server.
- Add an X-Plane menu item that opens the editor folder or docs page when the
  platform supports it; otherwise show the path.

Editor should provide:

- aircraft list and active config
- channel table with semantic role, PX4 source channel, dataref, type, index,
  input range, output range, reverse/sign, neutral, failsafe, clamp policy
- live validation results
- diff against PX4 airframe actuator params where possible
- actuator sweep preview
- import/export
- save with backup
- clear warnings for unknown datarefs or stale channels

The editor should be professional and quiet: dense tables, inline validation,
simple diagrams, no decorative dashboard layout.

## X-Plane Menu And UI Improvements

Recommended menu/workflow changes:

- Show active airframe and config validation state in the menu.
- Add `Reload and Validate Config`.
- Add `Open Config Editor`.
- Add `Open Docs`.
- Add a `Diagnostics` submenu with:
  - bridge health logging
  - sensor timing debug
  - sensor value debug
  - EKF/estimator debug
  - accel pipeline debug
- Add a compact status window showing:
  - connection state
  - active airframe
  - actual sensor/GPS/state/RC Hz
  - p50/p95 FPS
  - late/missed counts
  - last actuator age
  - config validation errors

Normal users should see minimal status. Developer diagnostics should stay
available but not flood X-Plane `Log.txt` by default.

## Diagnose Mode Explanation

There is not one single isolated "diagnose mode" today. The current project has
diagnostic flags in `config/config.ini`:

- `debug_verbose_logging`
- `debug_log_sensor_timing`
- `debug_log_sensor_values`
- `debug_log_ekf_innovations`
- `debug_log_accel_pipeline`
- `debug_accel_bypass_calibration`
- `diagnostic_log_enabled`
- `diagnostic_log_interval_s`
- `fps_warning_enabled`

The most important one for normal validation is `diagnostic_log_enabled`. It
writes compact bridge health lines into X-Plane `Log.txt`, including:

- effective sensor/GPS/state/RC send rates
- frame period min/p50/p95/max
- late/missed send counts
- timestamp monotonicity/cap counters
- local velocity
- accelerometer mapping values
- heading, magnetic heading, IAS/TAS

Normal release recommendation:

- Keep compact bridge diagnostics available.
- Default interval should be moderate, such as 2-5 seconds.
- Keep high-volume debug flags off by default.
- Add documentation explaining which flags are safe for normal users and which
  are development-only.
- Rename or group these settings so users understand the difference between
  normal health logging and deep debug logging.

## Documentation Structure

Add a cleaner docs index:

- `docs/index.md`
- `docs/user/install.md`
- `docs/user/run-xplane-sitl.md`
- `docs/user/airframes.md`
- `docs/user/troubleshooting.md`
- `docs/user/diagnostics.md`
- `docs/developer/architecture.md`
- `docs/developer/sensor-contract.md`
- `docs/developer/config-schema.md`
- `docs/developer/replay-tests.md`
- `docs/developer/release-packaging.md`
- `docs/airframes/alia250.md`
- `docs/airframes/ehang184.md`
- `docs/airframes/cessna172.md`
- `docs/airframes/tb2.md`
- `docs/airframes/qtailsitter.md`

The UI and README should link into these pages instead of duplicating long
instructions everywhere.

## Proposed Next Phases

### Phase 1 - Retest Baseline

You run v3.4.6 exactly with reset/distclean and return complete artifacts.

### Phase 2 - Analysis Report

I produce a ULog/truth-capture report with plots or CSV summaries and identify
which control loop limits the demo quality.

### Phase 3 - Config Safety Slice

Fix the X-Plane-side mapping hazards:

- clear stale channels
- startup load
- validation
- clamping
- actuator freshness
- bounded MAVLink drain
- tests

### Phase 4 - Alia Param Template Slice

Clean the Alia airframe file without changing behavior, then add parameter
metadata and traceability.

### Phase 5 - Alia Tuning Slice

Tune from logs, one phase at a time, with v3.4.6 as baseline.

### Phase 6 - UI/Docs Slice

Build the config editor, docs index, menu links, diagnostics docs, and user run
cards.

### Phase 7 - Release Candidate

Package:

- px4xplane plugin
- XPlaneTruthCapture plugin
- PX4 fork airframe files
- Alia test card
- config validation report
- known limitations

Then run the final Alia demo scenario.

## Approval Questions

Before implementation, decide:

1. For the next lab test, do we agree to use v3.4.6 baseline with
   reset/distclean and no new tuning?
2. Should Phase 3 config safety come before more Alia tuning? My recommendation
   is yes, because bad actuator mapping can masquerade as bad tuning.
3. For the future editor, do we agree to introduce a typed schema first and
   keep INI as generated/backward-compatible output?
4. Should normal releases keep compact diagnostics enabled by default? My
   recommendation is yes, with high-volume debug flags off.
