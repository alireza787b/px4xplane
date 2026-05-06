# Alia X-Plane 12 Validation Workflow

This workflow keeps the v3.4.3 bridge sensor contract as the baseline. Do not
change accel signs, GPS velocity units, GPS COG, or HIL_STATE units from a single
visual symptom. First compare truth capture, decoded bridge replay, and PX4 ULog
estimator topics.

## References

- PX4 Simulator MAVLink API: https://docs.px4.io/main/en/simulation/
- PX4 Generic Standard VTOL / QuadPlane tuning: https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration.html
- PX4 fixed-wing TECS/NPFG tuning: https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 multicopter PID tuning: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html

## Scenario

1. Install the px4xplane package with:
   - `px4xplane/64/win.xpl`
   - `px4xplane/64/config.ini`
   - `px4xplane/px4_airframes/5020_xplane_alia250`
   - this test note or an equivalent run card
2. Install XPlaneTruthCapture `v0.1.6` or newer. Confirm that starting capture
   changes the menu state to `Recording Active`; the optional overlay can be
   hidden with `overlay_enabled = false` in its `capture_config.ini`.
3. Use X-Plane 12 Alia with model calculations per frame set to `6`.
4. Start XPlaneTruthCapture before connecting PX4.
5. Run PX4 SITL with the Alia airframe.
6. Fly:
   - takeoff to about `100 m`
   - forward transition
   - RTL, circle, or loiter return
   - back-transition
   - landing and disarm
7. Wait `10-15 s` after disarm before stopping PX4. This makes shutdown-only
   vertical velocity warnings distinguishable from in-flight estimator issues.
8. Save:
   - XPlaneTruthCapture zip/folder
   - PX4 `.ulg`
   - PX4 CLI log
   - X-Plane `Log.txt`

## Replay

Decode truth-capture frames into bridge-contract rows:

```bash
python3 tools/replay_truth_capture.py /path/to/XPlaneTruthCapture-run.zip \
  --output /tmp/alia_bridge_replay.csv
```

This replay is a deterministic core-contract check. It intentionally omits
stochastic sensor noise so it can catch sign, unit, velocity, and scheduling
regressions before a full X-Plane/PX4 run.

For quick inspection:

```bash
python3 tools/replay_truth_capture.py /path/to/XPlaneTruthCapture-run.zip \
  --messages sensor,gps,state \
  --max-messages 20 \
  --format jsonl
```

## ULog Analysis

Install `pyulog` if needed:

```bash
python3 -m pip install pyulog
```

Analyze the estimator topics and warning windows:

```bash
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg --json > /tmp/alia_ulog_summary.json
```

The script requests these topics:

`estimator_status`, `estimator_status_flags`,
`estimator_innovation_test_ratios`, `estimator_aid_src_baro_hgt`,
`estimator_aid_src_gnss_vel`, `vehicle_local_position`,
`vehicle_air_data`, `sensor_baro`, `vehicle_gps_position`,
`commander_state`.

## Acceptance

- No in-flight EKF warnings.
- No GPS rejection bursts in estimator innovation/test-ratio topics.
- No barometer source switch during flight, or a clear ULog-backed explanation.
- Post-landing vertical velocity warning is eliminated or shown to be shutdown-only.
- Alia RTL/circle altitude and path deviation improve against the previous run.
