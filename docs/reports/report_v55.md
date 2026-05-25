# Report v55 - qtail19 Baro Liveness and Landing Contact

Date: 2026-05-25
Package target: `v3.4.43`

## Executive Summary

qtail19 confirms that v3.4.42 fixed the previous stationary high-accelerometer
bias, vertical-velocity instability, and pause/unpause estimator failures. The
new remaining stationary warning is different: PX4 reported `BARO #0 failed:
STALE!` on QuadTailsitter before arming.

Root cause: the v3.4.42 stationary-ground contract froze baro pressure exactly.
PX4's sensor `DataValidator` marks a sensor stale after more than 100 identical
samples, so a perfectly constant simulated pressure stream looks like a stuck
barometer even though packets are still arriving.

## Evidence Reviewed

From `/home/alireza/qtail19.zip`:

- px4xplane loaded `v3.4.42`.
- TruthCapture recorded `25,155` frames with zero dropped rows.
- Pause/unpause produced `[BRIDGE_PAUSE]` and `[BRIDGE_RESUME]` lines and did
  not produce the old high accel-bias or vertical-velocity failure.
- QuadTailsitter ULog showed `BARO #0 failed: STALE!` at about `4.6`, `19.6`,
  `34.5`, and `46.3 s`, all before arming.
- Alia did not show the same PX4 warning in ULog, but both logs showed
  simulated `sensor_baro` publication near `1 Hz`; the qtail stationary exact
  pressure made the stale detector trip first.
- QuadTailsitter later hit roll attitude failure during the landing/contact
  phase. Motor outputs were not generally saturated, so this log does not prove
  a thrust deficit. The current ACF contact model still had retractable long
  gear, large strut travel, and tiny `0.01` contact tires.

## Implemented Changes

- Kept the zero-motion stationary-ground contract for GPS, HIL_STATE, gyro, and
  acceleration.
- Re-enabled tiny deterministic/live barometer pressure variation while the
  stationary contract is active, so the simulated baro remains a live sensor
  instead of a mathematically frozen value.
- Updated TruthCapture replay to model the same stationary baro liveness and
  added a unit test proving stationary GPS altitude remains latched while baro
  pressure changes.
- Tuned the QuadTailsitter ACF landing contact model:
  - fixed the four gear supports so they are not retractable
  - increased contact tire/pad radius and width
  - reduced excessive strut compression travel
  - increased damping and support force
- Smoothed QuadTailsitter MC lateral commands conservatively by reducing
  horizontal acceleration and jerk.
- Set `LNDMC_Z_VEL_MAX` below the landing crawl/speed thresholds to avoid PX4
  startup correction noise.

## Next Test

Use `v3.4.43` and pull the updated PX4 PR branch airframe file.

1. QuadTailsitter stationary test for `90-120 s`, including one pause/unpause.
2. Arm, wait `5 s`, disarm. Stop if any baro stale, vertical velocity, accel
   bias, or compass warning appears.
3. If clean, run MC-only: takeoff, hover, two Go-To commands, RTL, and land.
4. Repeat a short stationary sanity check on Alia to confirm no regression.

Do not start fixed-wing transition again until QuadTailsitter MC Go-To, RTL, and
landing are clean with this contact model.

## Verification

- `python3 -m unittest tests/test_replay_truth_capture.py`: passed, `5/5`.
- `tools/analyze_qtailsitter_design.py` still reports the intended `4.99 kg`
  empty mass, `5.67 kg` max mass, `0.509 m^2` wing estimate, and `9.8 in`
  three-blade props.

