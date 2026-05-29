# Report v75 - PX4 Main Startup Cleanup and Alia Mission Review

Date: 2026-05-29

Package target: `v3.4.63`

## Evidence Reviewed

- User terminal paste from the clean-loading Alia run.
- `/home/alireza/17_21_30.ulg`
- PX4 upstream startup scripts and logger source after rebasing onto current
  `upstream/main`.

## Findings

1. Alia was the active PX4 airframe.
   - `SYS_AUTOSTART=5020`
   - `VT_TYPE=2`
   - `MAV_TYPE=22`
   - `CA_AIRFRAME=2`

2. The X-Plane airframe defaults were loaded, then overwritten by the common
   MAVLink simulator startup path.
   - Expected from X-Plane airframes: `SENS_GPS0_DELAY=0`,
     `SENS_GPS1_DELAY=0`, `EKF2_MULTI_IMU=1`, `IMU_INTEG_RATE=200`.
   - Logged in the run: `SENS_GPS0_DELAY=10`, `SENS_GPS1_DELAY=10`,
     `EKF2_MULTI_IMU=3`, `SENS_IMU_MODE=0`, `IMU_INTEG_RATE=250`.
   - Root cause: `px4-rc.simulator` and `px4-rc.mavlinksim` run after the
     airframe file and apply classic MAVLink simulator defaults.

3. The logger warning flood is real startup noise, not an airframe selection
   bug.
   - Current PX4 SITL default logging expands many estimator multi-instance
     topics.
   - The logger has a fixed subscription budget of 255 topics.
   - The broad default SITL logging profile (`SDLOG_PROFILE=131`) plus
     estimator multi-instance expansion exceeded that budget, producing many
     `Too many subscriptions` warnings.

4. `Preflight Fail: no heading reference` was transient.
   - EKF yaw alignment was initially false, then became valid before takeoff.
   - PX4 reported `Ready for takeoff` at about `5.3 s`.
   - This warning should be less likely after the X-Plane timing defaults stop
     being overwritten, but a short startup heading wait is normal if the
     estimator has not yet aligned.

5. The in-flight compass warning was short but real.
   - PX4 logged `Compass needs calibration - Land now!` and
     `Compass 0 fault` around `146.8 s`.
   - The logged magnetometer aid source had no innovation rejection and low
     test ratios, so this looked like estimator/health state churn rather than
     a gross magnetic data failure.
   - The warning cleared by about `180.3 s`.

6. The mission did not descend during the apparent orbit because the mission
   item was an explicit `NAV_LOITER_TO_ALT`.
   - Sequence `8` was `MAV_CMD_NAV_LOITER_TO_ALT` (`31`) with explicit
     `loiter_radius=75 m`.
   - That explicit mission radius bypasses the Alia default
     `NAV_LOITER_RAD=2000 m`.
   - The setpoint remained near the current fixed-wing altitude instead of
     descending to the intended landing altitude, so the aircraft appeared to
     loiter for a long time.
   - For the final Alia test, use a large VTOL landing/loiter radius in QGC;
     do not use the default small QGC pattern radius for this aircraft.

## Changes

- Marked all X-Plane PX4 targets with `PX4_SIMULATOR=xplane`.
- Changed the common simulator startup so X-Plane keeps its airframe-specific:
  - GPS delay defaults
  - IMU topology defaults
  - IMU integration rate default
- Limited SITL default estimator logger expansion to the three estimator
  instances used by the common mavlink simulator path instead of requesting all
  possible optional instances.
- Set all five X-Plane PX4 airframes to the default logging topic profile:
  `SDLOG_PROFILE=1`.
- Synchronized the packaged px4xplane `config/px4_params/*` reference files
  with the PR airframes.

## Expected Next Test

On the next Alia run, the ULog initial parameters should show:

```text
SYS_AUTOSTART=5020
SENS_GPS0_DELAY=0
SENS_GPS1_DELAY=0
EKF2_MULTI_IMU=1
IMU_INTEG_RATE=200
SDLOG_PROFILE=1
```

The terminal should no longer print the large `Too many subscriptions` logger
warning block. A short early heading-reference wait is acceptable only if it
clears before arming and PX4 reports `Ready for takeoff`.

## Final-Test Note

If testing full mission landing, set the QGC VTOL landing/loiter geometry for a
large Alia-scale pattern. A `75 m` fixed-wing loiter radius is not compatible
with the tuned Alia envelope and can make the aircraft appear stuck before the
landing/back-transition stage.
