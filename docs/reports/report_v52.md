# Report v52 - Ground-Stationary IMU Guard

Date: 2026-05-25
Package target: `v3.4.40`

## Executive Summary

The new failure is a bridge sensor robustness issue, not a QuadTailsitter tuning
result. The uploaded X-Plane truth data shows the aircraft remained stationary
on the ground with zero motor output, but X-Plane reported large gear/contact
g-load spikes. px4xplane v3.4.39 forwarded those spikes directly as HIL_SENSOR
accelerometer data, which can trigger PX4 vertical velocity instability as soon
as SITL connects.

v3.4.40 adds a default-on stationary-ground accelerometer guard. While the
aircraft is on the ground, very low AGL, and effectively not moving, the bridge
publishes stable gravity instead of X-Plane contact impulses. The guard
disengages once the aircraft is airborne or moving.

## Evidence Reviewed

Input files:

- `/home/alireza/Log.txt`
- `/home/alireza/20260525-044651Z/Log.txt`
- `/home/alireza/20260525-044651Z/frames.csv`
- `/home/alireza/20260525-044651Z/datarefs.csv`

TruthCapture summary:

- frames: `10432`
- sim time: `5.300978184 s` to `134.2634277 s`
- paused: always `0`
- on-ground flag: always `1`
- AGL: about `0.033 m` to `0.037 m`
- groundspeed: p50 about `0.006 m/s`, max about `0.230 m/s`
- throttle-used: `0`
- prop rotation speed: `0`
- prop thrust: `0`

The acceleration evidence was inconsistent with a stationary aircraft:

- `g_nrml`: p50 about `0.572 g`, p95 about `1.833 g`, max about `6.291 g`
- `local_ay`: min about `-7.40 m/s^2`, p95 about `8.17 m/s^2`, max about
  `51.85 m/s^2`
- `abs(g_nrml) > 1.5 g`: `2208` frames
- `abs(local_ay) > 5 m/s^2`: `4186` frames

That is X-Plane ground-contact jitter. It is not flight dynamics, motor thrust,
or an aircraft-parameter effect.

## Implemented Changes

- Added `ground_stationary_accel_guard_enabled = true` to `config.ini`.
- Added the same field to `config_schema.json` under `sensor_contract`.
- In `MAVLinkManager::computeAcceleration()`, detect stationary-on-ground state
  using:
  - `sim/flightmodel/failures/onground_any`
  - `sim/flightmodel/position/y_agl`
  - local vertical speed
  - local horizontal speed
- When the guard is active, replace X-Plane g-load acceleration with
  `[0, 0, -9.80665] m/s^2` in PX4 FRD.
- Reduced repeated `config.ini` path log spam by logging the resolved config
  path only when it changes.

## Test Instruction

Before resuming QuadTailsitter tuning, run a stationary bridge sanity check:

1. Install v3.4.40.
2. Start X-Plane and load the intended aircraft.
3. Connect PX4 SITL while the aircraft is stationary on the ground.
4. Confirm X-Plane `Log.txt` contains:

   ```text
   px4xplane: Version: v3.4.40
   px4xplane: Ground-stationary accelerometer guard enabled
   px4xplane: [GROUND_ACCEL_GUARD]
   ```

5. PX4 should not immediately report persistent vertical velocity instability.
6. Only after this stationary check is clean should QuadTailsitter MC testing
   resume.

If the warning still appears, collect a ULog plus X-Plane truth data from the
stationary connection case before changing aircraft tuning.
