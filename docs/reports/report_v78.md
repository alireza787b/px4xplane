# Report v78 - Alia EKF Yaw Reset Recovery

Date: 2026-05-30

Package target: `v3.4.66`

## Evidence Reviewed

- `/home/alireza/newAlia2.zip`
  - `02_17_26.ulg`
  - `02_20_42.ulg`
  - TruthCapture `20260530-022007Z/frames.csv`
  - X-Plane `Log.txt`
- PX4 EKF2 magnetometer, GNSS, and yaw-estimator source on the current PR
  branch.
- PX4 documentation:
  - EKF2 magnetometer fusion modes:
    https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
  - `sensor_gps.timestamp_sample` contract:
    https://docs.px4.io/main/en/msg_docs/SensorGps
  - VTOL back-transition tuning:
    https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning

## Findings

The short first-run `Preflight Fail: ekf2 missing data` was a startup warm-up
message. In the completed Alia run, PX4 reported `Ready for takeoff` and the
flight completed. This is not the release-blocking issue.

The release-blocking compass warning was not caused by bad raw magnetometer
data. At the first warning, the magnetometer vector was plausible and the
ground-truth yaw, GPS course-over-ground, and raw magnetic heading agreed near
`-74 deg`. The main EKF yaw instead jumped to about `-50 deg`.

The failure chain in `02_20_42.ulg` was:

1. Front transition started at about `65.3 s`.
2. With `SENS_GPS0_DELAY=110` and `SENS_GPS1_DELAY=110`, PX4 compared current
   X-Plane HIL GPS positions against an over-delayed estimator state.
3. GNSS position innovations grew during the acceleration phase. Examples:
   - around `85 s`: GPS position innovation about `[-7.1, 18.2] m`
   - around `100 s`: GPS position innovation about `[-9.4, 22.7] m`
4. PX4 EKF2 treated this as a yaw failure and reset yaw to the EKF-GSF yaw
   estimate at about `89.5 s` and again near `107.0 s`.
5. The EKF-GSF yaw estimate was wrong during this transient. The first reset
   moved yaw from about `-75 deg` to about `-50 deg`.
6. PX4 then marked the magnetometer faulty because the emergency yaw-reset path
   explicitly faults active mag fusion after a yaw rescue.

The previous v77 decision to use the full `110 ms` fallback GPS delay was too
large for px4xplane. X-Plane HIL GPS samples are current-position samples. The
PX4 `vehicle_gps_position` module still needs a non-zero delay when the driver
does not populate `timestamp_sample`, but for this simulator contract the common
mavlinksim `10 ms` value is the right scale.

PX4's EKF2 documentation also supports using magnetic-heading fusion for this
SITL case. Automatic mag fusion switches to 3D fusion in flight, which can
affect tilt. The X-Plane simulated magnetic field has no real hard-iron learning
requirement, so heading-only fusion is the cleaner simulator default.

## Changes

Updated all five PX4 PR X-Plane airframes and the matching px4xplane reference
copies:

- `5001_xplane_cessna172`
- `5002_xplane_tb2`
- `5010_xplane_ehang184`
- `5020_xplane_alia250`
- `5021_xplane_qtailsitter`

Sensor-contract changes:

```text
SENS_GPS0_DELAY: 110 -> 10
SENS_GPS1_DELAY: 110 -> 10
EKF2_MAG_TYPE:   0   -> 1
```

This is not a warning suppressor. It keeps GPS timing physically consistent with
the bridge and keeps the simulated magnetometer as a heading source while
avoiding automatic 3D mag/tilt interaction during transition.

No Alia back-transition parameters were changed in this slice. The current
back-transition still uses:

```text
VT_B_TRANS_DUR=35.0
VT_B_DEC_MSS=1.5
VT_B_DEC_I=0.25
VT_B_TRANS_RAMP=15.0
```

The ULog shows the aircraft was still under the earlier estimator fault history
before back-transition. The accepted v3.4.22 evidence specifically supported
`VT_B_TRANS_RAMP=15.0`, so changing it now would mix two variables in the next
test. Retest the estimator fix first; only retune back-transition if the pitch
oscillation remains with no compass/yaw fault.

## Next Test

Use a clean PX4 parameter store once after pulling this branch:

```text
px4xplane --sync --reset-config
```

Then select Alia and choose `d` for `distclean` in the launcher.

Before judging the flight, confirm the ULog initial parameters contain:

```text
SYS_AUTOSTART=5020
SENS_GPS0_DELAY=10
SENS_GPS1_DELAY=10
EKF2_MAG_TYPE=1
EKF2_MULTI_IMU=1
SENS_IMU_MODE=0
IMU_INTEG_RATE=200
```

Expected result:

- transient startup `no heading reference` or `ekf2 missing data` may appear
  before `Ready for takeoff`, but must clear before arming readiness.
- no in-flight `Compass 0 fault`
- no transition-time emergency yaw resets
- no quad-chute in front transition
- back-transition should be reassessed only after the yaw/compass fault is gone
