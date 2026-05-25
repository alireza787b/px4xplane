# Report v54 - Stationary-Ground Sensor Contract

Date: 2026-05-25
Package target: `v3.4.42`

## Executive Summary

v3.4.41 was still incomplete. It zeroed local velocity while stationary on the
ground, but PX4 was still receiving a mixed sensor story: raw GPS/baro altitude
noise, raw gyro/contact rates, and a level-only gravity replacement that could
be wrong when the aircraft was tilted after a contact event.

This slice turns the two guard flags into one coherent stationary-ground sensor
contract. While X-Plane reports the aircraft on the ground, near the ground, not
moving horizontally, and without meaningful mapped motor command, px4xplane now
publishes a self-consistent zero-motion state to PX4. It is still general
bridge logic, not a QuadTailsitter-specific tuning workaround.

## Evidence Reviewed

From `/home/alireza/qtail17.zip`:

- `05_53_03.ulg` showed stationary warnings before TruthCapture indicated
  airborne flight. PX4 logged `high accel bias` and `vertical velocity
  unstable` while motor outputs were still disarmed and the bridge was already
  publishing zero GPS velocities and stable acceleration.
- The remaining inconsistency was altitude/rate related: GPS/baro altitude
  still had raw/noisy samples and gyro rates still came from X-Plane contact
  dynamics. EKF time slip also grew in that stationary period, so this remains a
  timing area to watch, but the first fixable fault was the inconsistent sensor
  contract.
- `06_00_34.ulg` showed why the previous level-only gravity replacement was too
  broad: after a crash/tip state, publishing `[0, 0, -g]` regardless of actual
  attitude can conflict with magnetometer and attitude. The replacement must be
  attitude-consistent.

The QuadTailsitter landing gear/contact model still looks like an amplifier:
long angled gear, large strut compression values, and very small contact/tire
dimensions are likely to create more X-Plane ground jitter than Alia. I did not
change the ACF in this bridge slice because the bridge must first behave
correctly for any stationary aircraft.

## Implemented Changes

- Added a latched `GroundSensorContractState` in the bridge.
- Activation now requires:
  - X-Plane on-ground flag
  - AGL within the ground-contact band
  - low horizontal speed
  - no fresh mapped motor command above a small thrust threshold
- Release now uses hysteresis and happens when the aircraft moves, lifts off, or
  mapped motors command meaningful thrust.
- While active, px4xplane now:
  - publishes attitude-consistent gravity in HIL_SENSOR acceleration
  - zeros HIL_SENSOR gyro rates
  - zeros GPS and HIL_STATE local velocities
  - latches GPS/HIL_STATE latitude, longitude, and altitude
  - latches barometer altitude and suppresses baro noise
  - suppresses GPS altitude noise
  - uses the guarded position when deciding whether to update the WMM magnetic
    field
- Resetting MAVLink state also resets the ground contract so aircraft changes
  or reconnects cannot reuse an old latched ground point.
- Updated `tools/replay_truth_capture.py` so offline truth replay can apply the
  same stationary-ground contract.
- Added a replay unit test proving that stationary contact acceleration/rate
  spikes are masked while position/velocity are held coherently.

## What This Does Not Do

- It does not synthesize high-rate IMU interpolation. We are still keeping one
  sensor packet per X-Plane frame until replay evidence proves interpolation
  improves PX4 estimator behavior.
- It does not tune QuadTailsitter landing gear or aircraft design. That remains
  the next slice after the stationary connect test is clean.
- It does not hide real flight dynamics. The contract is gated to stationary
  ground contact and releases on lift, motion, or motor command.

## Verification

Commands run:

```text
python3 -m unittest tests/test_replay_truth_capture.py
ctest --test-dir build-offline-tests --output-on-failure
cmake --build build-offline-tests -- -j$(nproc)
cmake --build build-win-mingw -- -j$(nproc)
python3 tools/replay_truth_capture.py /home/alireza/qtail17.zip --messages sensor,gps,state --max-messages 6 --format jsonl
```

Results:

- Replay tests: 4/4 passed.
- CTest: 5/5 passed.
- Linux package build: passed.
- Windows package build: passed.
- Replay smoke on `qtail17.zip`: stationary outputs showed zero gyro, zero
  GPS/state velocity, fixed altitude, and attitude-consistent gravity.

## Next Test Instruction

Install `v3.4.42` and do not resume QuadTailsitter tuning immediately. First do
this short bridge sanity test:

1. Load QuadTailsitter.
2. Start X-Plane unpaused and stationary.
3. Start PX4 SITL and connect.
4. Wait 90-120 seconds on the ground without arming.
5. Pause/unpause once for a few seconds.
6. If clean, arm, wait 5 seconds, then disarm without takeoff.
7. Repeat the same stationary test once on Alia.

Expected X-Plane `Log.txt` lines:

```text
px4xplane: Version: v3.4.42
px4xplane: Ground-stationary accelerometer contract enabled
px4xplane: Ground-stationary kinematics contract enabled
px4xplane: [GROUND_SENSOR_CONTRACT] Stationary ground state latched
px4xplane: [GROUND_ACCEL_GUARD] Stationary on ground; replacing X-Plane contact g-loads with attitude-consistent gravity
```

Acceptance before returning to QuadTailsitter tuning:

- No PX4 `vertical velocity unstable` while stationary.
- No `High Accelerometer Bias` while stationary.
- No compass fault from pause/unpause.
- TruthCapture still shows any physical X-Plane contact jitter, but replayed
  bridge outputs show the stationary zero-motion contract.

If any warning remains, collect ULog, X-Plane `Log.txt`, and TruthCapture for
only the stationary test. Do not change aircraft aero/gear tuning until this
bridge sanity gate is clean.
