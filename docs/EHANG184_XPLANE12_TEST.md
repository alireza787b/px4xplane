# EHang 184 X-Plane 12 Test Card

Use the matching EHang model, plugin section `ehang184`, PX4 target
`xplane_ehang184` and `SYS_AUTOSTART=5010`. This is an MC-only workflow:
do not apply Alia trims, fixed-wing gains or VTOL transitions.

## Setup

1. Follow [normal setup](../README.md#quick-start) or the
   [guided launcher](guided-launcher.md). Use the current released plugin;
   reload X-Plane after a binary replacement.
2. Load EHang. Record actual aircraft path, mass/CG, weather, X-Plane version
   and physics/render settings. Model dimensions and motor order must be
   checked from the loaded ACF, not inherited comments or brochure figures.
3. Select **Plugins > PX4 X-Plane > Airframes > Ehang 184**, then
   **Connect to SITL**. The configuration section remains `ehang184`.
4. Start official PX4, selecting `xplane_ehang184` in the guided launcher:

   ```bash
   px4xplane --official
   ```

   Manual equivalent after the documented host/network setup:

   ```bash
   make px4_sitl_default xplane_ehang184
   ```

5. Check `ver all`, `param show SYS_AUTOSTART` and `mavlink status`.
   QGC must display a connected vehicle; transmitted bytes alone are not proof.
   Reconnect the plugin if the first build exceeded its connection timeout.

Normal setup needs no pending PR. `--validation` selects a separate TECS
proposal and is not an EHang tuning mode. `--reset-config` resets launcher
choices, not parameters. After reviewing changed defaults, explicitly use
`--reset-params` to back up/reset only the selected isolated runtime.

## First Controlled Flight

Keep calm weather and unchanged loading/CG. Verify engine index, rotation and
command response against PX4 allocation before flight; do not reverse motors
because a diagram/comment uses inconsistent axis labels.

1. Set takeoff height to **30 m** and take off. Hover **30 s** after settling.
2. Send one modest GoTo about **150-200 m** ahead at the same altitude.
   Wait for capture/settling, then another point roughly perpendicular.
3. Make a gentle yaw change if control is available. Avoid simultaneous manual
   inputs that make a response impossible to attribute.
4. If the preceding stages are stable, command a climb to **50 m**, settle,
   then descend to **30 m**. These are simulator test points, not certified limits.
5. If the preceding stages are stable, use Return and observe landing/disarm.
   If Return does not land, record the state and use Land nearby; that is not
   a passing RTL result. Wait another **15 s** after disarm before `shutdown`.

For console takeoff, set the scenario height before `commander takeoff`;
the bare command uses the current MIS_TAKEOFF_ALT, which may not be 30 m.
Use `commander land` for local landing. QGC Hold corresponds to
`commander mode auto:loiter`, not `commander mode hold`.

A QGC GoTo acknowledgement alone is not tracking evidence. Check the actual
navigation mode and position setpoint. If reposition is not followed, record it
and Land rather than retuning attitude gains to solve a mode-selection problem.
Defer Orbit, higher speeds and wind/CG tests until this baseline is reviewed.

## Evidence And Review

For diagnostic flights, start TruthCapture after loading EHang and before
connecting PX4; confirm a new growing recording. Capture the same flight's:

- ULog, source revision, effective initial parameters and later parameter changes.
- Plugin version/binary identity, config.ini, loaded-model identity and Log.txt.
- Independent attitude/rates, position/velocity, available forces, mass and CG.
- Commands, actual navigation states, controller setpoints, motor outputs,
  allocator residuals and landing/disarm state.

Optional owner automation is not required for public setup. Do not publish
proprietary model files or upload credentials with review plots.

Tune rate/attitude behavior before position/trajectory limits. Compare demanded
and measured response and actuator margin; do not transfer Alia gains because
both vehicles are described as air taxis. Keep smoothness goals separate from
claims of passenger comfort or certification.

```bash
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg --json
python3 tools/check_px4_airframe_params.py /path/to/log.ulg /path/to/tested/5010_xplane_ehang184 --json
```

Names/defaults depend on PX4 revision; the source airframe is authoritative,
including inherited values. No duplicated fixed parameter checklist is maintained
here. The packaged EHang reference matches official `95a0c2348232`; this is
source synchronization, not acceptance of a new EHang flight.

## Acceptance

- No unexplained in-flight estimator fault, rejection/reset or bias warning.
- Damped attitude/rate and position/altitude capture, without sustained clipping.
- Verified command/mode transitions and actual motion toward valid setpoints.
- Controlled descent, touchdown and automatic disarm.
- Complete same-run ULog/truth evidence with no unexplained gaps/time resets.

Retain failed and low-FPS cases for diagnosis; do not increase bias/arming
thresholds to force a pass. One passing flight does not establish every payload,
CG, weather, speed or frame-rate condition.
