# px4xplane Alia Validation Follow-Up Report v12

Date: 2026-05-06

Scope: restored the project workflow after context loss, reviewed the successful
X-Plane 12 Alia run, audited the first v3.4.3 follow-up slice, and completed the
missing XPlaneTruthCapture UI/release work.

## Summary

The May 6 Alia run is the strongest validation so far of the v3.4.3 sensor
contract. PX4 completed takeoff, forward transition, RTL, back-transition,
landing, disarm, and ULog close. The remaining work is now tuning,
observability, and staged architecture cleanup, not another broad sign/axis
rewrite.

## Implemented

- px4xplane bridge diagnostics for effective stream rate, frame timing,
  late/missed sends, timestamp counters, and target-rate warnings.
- Deterministic XPlaneTruthCapture replay tool for core MAVLink sensor contract
  checks.
- ULog estimator analyzer for requested estimator topics and event windows.
- Dataref provider seam for future X-Plane-independent sensor generation tests.
- Alia parameter cleanup and conservative TECS/NPFG/path-following first pass.
- Release-default log noise reduction while keeping compact diagnostics enabled.
- Waiting-socket disconnect cleanup when PX4 connection wait is cancelled.
- Removal of outgoing HIL_SENSOR timestamp jitter to keep same-frame timestamps
  globally monotonic.
- XPlaneTruthCapture v0.1.6 recording-active menu state, configurable overlay,
  and generic-marker fix.

## Verification

- `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests/test_replay_truth_capture.py`
- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/replay_truth_capture.py tools/analyze_ulog_estimator.py tests/test_replay_truth_capture.py`
- `git diff --check`
- `cmake --build build`
- `cmake --build build-win-mingw`
- `cmake --build build --config Release` in `/home/alireza/xplane-truth-capture`
- ULog analyzer ran on `/home/alireza/alia-sitl1/05_17_33.ulg` with temporary
  `pyulog`; all requested estimator topics were present except `commander_state`.

## Current Access Gap

The PX4 fork referenced by the SITL CLI log, `/home/alireza/PX4-Autopilot-Me`,
is not currently present locally. The updated Alia airframe file should be
synced there after the fork is restored or recloned.

## Next Test Package

The next handoff should include:

- px4xplane Windows plugin zip.
- XPlaneTruthCapture Windows plugin zip.
- standalone `5020_xplane_alia250` for PX4 fork overwrite.
- run card telling the tester to wait `10-15 s` after disarm before stopping PX4.
