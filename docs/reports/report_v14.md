# Report v14 - Offline Work Plan, UI Direction, Prop Brake Strategy

Date: 2026-05-07

Scope: answer the current discussion questions, decide what can safely progress
while the GPU/X-Plane machine is unavailable, and define the next approval plan.
No runtime code, parameter, package, or release change is included in this report.

## Current Situation

The GPU/X-Plane machine may be unavailable until Monday. We should not wait
idle, but we should also not change flight behavior blindly.

Recommended policy:

- Freeze `px4xplane v3.4.6` flight behavior as the protected Alia baseline.
- Do offline work that improves architecture, tests, config safety, docs, and
  release reliability.
- Do not tune Alia, change sensor signs/units, change quaternion behavior, or
  redesign sensor-rate generation until a real X-Plane retest returns complete
  artifacts.

The next lab run should still use the v3.4.6 package and a PX4 parameter reset
or `distclean` before flight.

## Was Ehang Loaded By Accident Or Is This A Bug?

The uploaded bad-flight ULog `alia-sitl2/19_07_20.ulg` has
`SYS_AUTOSTART=5010`. In this project:

- `5010` = Ehang 184
- `5020` = Alia 250

So that uploaded ULog is not Alia evidence.

Most likely causes:

- the setup menu selection or run target launched `xplane_ehang184`
- saved PX4 SITL state or manual-copy workflow caused confusion about which
  airframe was actually active
- the log file came from a different run than the Alia retest

Current evidence does not prove that px4xplane randomly changed the PX4
airframe. It does show that we need stronger preflight sanity checks:

- PX4 CLI must print `SYS_AUTOSTART=5020` for Alia tests.
- The X-Plane plugin menu/status window should show active plugin airframe
  mapping.
- The test README/run card should say: stop if either side does not match Alia.

Action for the next implementation slice:

- add a clearer UI/config validation state, and possibly a warning when the
  selected X-Plane-side config and PX4 airframe expectation disagree.

## Should We Use The Waiting Time?

Yes. Use the no-GPU window for work that can be verified with local tests.

Safe offline now:

- organize reports and docs
- freeze and document sensor contracts
- strengthen replay/golden tests
- add config parser/schema validation tests
- fix stale config reload behavior
- add actuator output clamp/NaN/freshness logic behind tests
- improve build/package verification
- improve docs/run cards
- plan the config editor and UI changes
- audit Ehang params and mapping for obvious inconsistencies

Must wait for real X-Plane/PX4 retest:

- Alia tuning claims
- Ehang passenger-smooth claims
- prop-brake realism validation
- quaternion/baro/mag behavior changes
- final packaging loadability in X-Plane on Windows
- any conclusion about v3.4.6 flight quality

## Prop Brake / Windmilling Discussion

The concept is valid. In fixed-wing cruise, Alia-like lift rotors should not
freely windmill if that creates large drag, vibration, noise, or ugly visuals.
Braking or feathering inactive lift props is a reasonable simulation goal.

The current mechanism is fragile:

- `autoPropBrakes = 0, 1, 2, 3` enables Alia lift-motor braking.
- The code toggles X-Plane failure datarefs for engine seizure and prop
  separation when throttle is exactly `0.0`.
- That can create discontinuities, persistent failure state, log noise, and
  mode-transition artifacts.

Better strategy:

1. Keep the prop-brake concept.
2. Replace the failure-dataref trick with an explicit prop-brake policy.
3. Use hysteresis and dwell time, not exact `throttle == 0.0`.
4. Brake only lift motors, never the pusher.
5. Engage only after front transition is stable and lift-motor command/RPM are
   below thresholds.
6. Release before or during back-transition using a controlled ramp.
7. Prefer X-Plane prop feather / prop mode / prop pitch datarefs if the Alia
   model supports them.
8. If X-Plane cannot model this cleanly for the aircraft, controlled windmilling
   is safer for demos than simulated failure/prop-separation.

Relevant X-Plane references:

- Prop pitch/mode override behavior:
  https://developer.x-plane.com/article/understanding-override_prop_pitch-and-override_prop_mode/
- Propeller feathering systems:
  https://developer.x-plane.com/article/propeller-feathering-systems/

Decision: do not change this before the v3.4.6 retest unless we keep behavior
off by default or make the change purely safety/validation infrastructure.

## UI/UX Direction

Recommended split:

- normal users: native X-Plane SDK menu/window/overlay
- advanced users: external local HTML/JS config editor backed by a typed schema
- developer tools: Python command-line tools for replay, logs, schema generation,
  and reports

Do not build a full config editor in native X-Plane UI. X-Plane SDK UI is good
for menus, status, floating windows, commands, and overlays, but large editable
tables, validation flows, file dialogs, and docs browsing are better outside the
sim.

Do not make Python GUI the primary user UX. Packaging PyQt/Tk or similar across
Windows/macOS/Linux and X-Plane plugin install paths adds avoidable friction.

Recommended native X-Plane menu:

- `Connect to PX4 SITL` / `Disconnect from PX4 SITL` as one stateful item
- `Show Status Window`
- `Airframe > Alia250 *`
- `Reload Config`
- `Validate Config`
- `Diagnostics > Bridge Log On/Off`
- `Diagnostics > Open Log Location`
- `Help > Documentation`
- `Help > Custom Airframes`
- `Help > GitHub`

Recommended native windows/overlays:

- Keep the current native status window as an inspector, not an editor.
- Suggested tabs: `Connection`, `Vehicle`, `Sensors`, `Actuators`, `Mapping`.
- Add validation badges for missing datarefs, wrong type, invalid range, stale
  channels, stale HIL actuator input, and active airframe mismatch.
- Keep connection HUD and FPS warning small and transient.
- No large persistent overlays unless debug mode is explicitly enabled.

Recommended key-bindable commands:

- `px4xplane/toggle_connection`
- `px4xplane/show_status_window`
- `px4xplane/reload_config`
- `px4xplane/validate_config`
- `px4xplane/toggle_diagnostics`

X-Plane references:

- Menus and command-backed menu items:
  https://developer.x-plane.com/sdk/XPLMMenus/
- Custom commands:
  https://developer.x-plane.com/sdk/XPLMCreateCommand/
- Widgets/native windows:
  https://developer.x-plane.com/sdk/XPWidgets/

## Config Editor Recommendation

Build a schema-first editor, not an INI text editor.

Near-term:

- keep `config.ini` supported
- define a typed schema internally
- add validators and tests
- optionally generate `config.ini` from schema later

Long-term:

- move to `config.json` or `config.yaml` if the schema proves stable
- keep INI import/export for backward compatibility
- add a static HTML/JS editor in the repo
- let X-Plane reload/validate and show results

Minimum editor fields:

- airframe name
- PX4 expected autostart id
- channel id
- semantic role
- PX4 source channel/function
- dataref name
- dataref type
- array index
- input range
- output range
- reverse/sign
- neutral value
- failsafe value
- clamp policy
- prop-brake policy
- validation status

Minimum editor features:

- live syntax validation
- schema validation
- backup before save
- import/export
- preview scaled writes
- actuator sweep plan
- clear warnings on unknown datarefs/types/ranges
- diff/check against PX4 airframe output mapping when possible

## Full Architecture Rewrite Direction

Yes, we are still planning toward a production-level architecture. The approach
should be staged, not a single rewrite.

Target architecture:

- `XPlaneAdapter`: datarefs, menu/window, X-Plane flight loop
- `MavlinkTransport`: TCP/UDP/socket IO and MAVLink parse/drain
- `SensorContract`: pure builders for HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION
- `CoordinateTransforms`: FRD/NED/body conversions, heading/course helpers
- `EstimatorDiagnostics`: bridge health and session records
- `ActuatorMapper`: typed config to safe X-Plane writes
- `ConfigSchema`: parse, validate, migrate, serialize
- `AirframeRegistry`: PX4-side and X-Plane-side alignment metadata
- `ReplayHarness`: X-Plane-independent regression tests
- `Packaging`: CI build, dependency gates, release package validation

First rule: preserve v3.4.6 sensor behavior until tests prove equivalence.

Protected sensor contract:

- accel FRD mapping: `x=-g_axil`, `y=+g_side`, `z=-g_nrml`
- GPS NED velocity: `vn=-local_vz`, `ve=local_vx`, `vd=-local_vy`
- GPS COG from true local velocity track
- HIL_STATE unit fixes, especially cm/s and milli-g
- monotonic, non-jittered HIL_SENSOR timestamps

## Offline Work Plan Until Monday

### Slice 1 - Reports And Docs Organization

- Create `docs/reports/`.
- Add new reports there starting with this v14.
- Later move older root `report*.md` files into `docs/reports/archive/` with
  link preservation or an index.
- Add `docs/index.md` and a clear docs tree.

### Slice 2 - Sensor Contract Golden Tests

- Convert replay outputs into golden fixtures.
- Add stationary, climb/descent, north/east/south/west, low-speed COG hold, and
  airspeed conversion cases.
- Add tests that fail if protected signs/units regress.

### Slice 3 - Config Safety

- Clear stale actuator configs on reload/switch.
- Load config at plugin startup.
- Validate sections, types, ranges, array indices, and missing datarefs through
  an injectable/mock dataref registry.
- Clamp and NaN-check actuator outputs.
- Add actuator freshness timeout.
- Add bounded MAVLink receive drain per frame.

### Slice 4 - Prop-Brake Policy Design

- Add schema fields and docs for prop-brake policy.
- Add testable state machine design with hysteresis/dwell/ramp.
- Do not enable a new runtime mechanism until X-Plane model support is checked.

### Slice 5 - Build And Release Hardening

- Add CI/package verification.
- Add Windows dependency gate.
- Decide static runtime vs bundled runtime policy.
- Verify packages contain plugin binary, config, airframes, README, and run card.

### Slice 6 - Ehang Offline Audit

- Clean/sanitize comments that overclaim "production ready" or passenger quality.
- Verify motor/channel ordering against PX4 output functions and config.
- Define comfort metrics and run card.
- Do not claim Ehang passenger-grade until a real X-Plane/ULog/TruthCapture run
  proves it.

## Ehang 184 Direction

The current Ehang params are comfort-oriented on paper:

- low cruise speed
- low auto jerk
- gentle descent
- large acceptance radii
- conservative position gains
- smooth vehicle response
- slower takeoff ramp

But the comments overclaim confidence without enough current evidence. We can
audit and clean them offline. We cannot honestly tune Ehang to passenger-grade
without logs.

Relevant public reference for a modern EHang passenger aircraft:

- EH216-S official page lists 1.93 m height, 5.73 m width, 30 km range,
  130 km/h maximum design speed, and 620 kg MTOW:
  https://www.ehang.com/ehang216s/

Note: our current X-Plane airframe is named Ehang 184, not EH216-S, so public
EH216-S data is useful context but not proof of exact model behavior.

## Monday Lab Recommendation

If offline work lands before Monday, the Monday test should still be controlled:

1. First run Alia v3.4.6 baseline unchanged.
2. Collect complete artifacts.
3. Only after the baseline is confirmed, optionally test one additional package
   that contains offline safety changes but no flight-tuning changes.
4. Do not bundle fresh Alia tuning with config/mapping safety changes in the
   same first retest; otherwise we cannot isolate effects.

## Logo And Image Generation

Yes, image generation is available in this environment through the image tool.
I can generate logo concepts for:

- `px4xplane`
- `XPlaneTruthCapture`

Recommended logo process:

1. Define brand brief first: audience, tone, colors, constraints, where it will
   appear.
2. Generate several raster concept directions.
3. Select one or two.
4. Rebuild the chosen concept as clean SVG/vector manually or with design-tool
   assistance so it is usable in GitHub, docs, release assets, and UI.

Do not use a raw generated bitmap as the final logo source of truth. Use image
generation for exploration, then create a clean vector mark.

Suggested directions:

- px4xplane: precise bridge/trajectory mark, aircraft silhouette, PX4/X-Plane
  connection motif, restrained aerospace colors.
- XPlaneTruthCapture: recorder/data-trace motif, timeline/crosshair, minimal
  diagnostics feel.

Logo work can run in parallel later, but it should not block the current
engineering slices.

## Decisions Requested

Please review and decide:

1. Approve offline work now instead of waiting until Monday?
2. Approve the freeze: no Alia tuning/sensor behavior changes before the next
   v3.4.6 baseline retest?
3. Should config safety be the first implementation slice?
4. Should the future config editor be schema-first static HTML/JS with native
   X-Plane UI only for status/reload/validate?
5. Should we include Ehang offline audit now, but defer Ehang tuning claims until
   logs are available?
6. Do you want a separate logo brief/report before generating logo concepts?
