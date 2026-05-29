# Report v76 - X-Plane IMU Selection Recovery

Date: 2026-05-29

Package target: `v3.4.64`

## Problem

After the startup-cleanup slice, PX4 no longer printed the large logger warning
block, but preflight could show:

```text
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data
ERROR [sensors] Accel #0 fail:  TIMEOUT!
```

## Root Cause

The v3.4.63 cleanup correctly stopped common mavlinksim startup defaults from
overwriting X-Plane airframe-specific GPS delay, EKF multi-IMU count, logger,
and IMU integration-rate settings. That also removed the old implicit
`SENS_IMU_MODE=0` default that came from the common mavlinksim script.

Current PX4 defines `SENS_IMU_MODE` with default `1`:

```text
0: Disabled
1: Publish primary IMU selection
```

PX4 EKF2 multi-instance startup explicitly requires `SENS_IMU_MODE=0`. Every
X-Plane airframe sets `EKF2_MULTI_IMU=1`, so the airframes must also explicitly
set `SENS_IMU_MODE=0`. Without that pairing, the sensors module can use the
primary-IMU selection path before the single X-Plane HIL IMU stream is selected
cleanly, which matches the observed `Accel #0 TIMEOUT` and `ekf2 missing data`.

## Fix

- Added `param set-default SENS_IMU_MODE 0` to all PX4 X-Plane airframes:
  - `5001_xplane_cessna172`
  - `5002_xplane_tb2`
  - `5010_xplane_ehang184`
  - `5020_xplane_alia250`
  - `5021_xplane_qtailsitter`
- Mirrored the same files into `px4xplane/config/px4_params/`.
- Updated the current docs/test cards so the expected startup values include
  `SENS_IMU_MODE=0`.

## Expected Next Test

The next ULog initial parameters should include:

```text
EKF2_MULTI_IMU=1
SENS_IMU_MODE=0
SENS_GPS0_DELAY=0
SENS_GPS1_DELAY=0
IMU_INTEG_RATE=200
SDLOG_PROFILE=1
```

The terminal should not show repeated `Accel #0 fail: TIMEOUT!`. A short
initial `ekf2 missing data` warning is acceptable only while PX4 is still
waiting for the simulator connection and estimator startup; it must clear before
arming and before judging the flight.

## Local Sync Note

The PR branch has been force-updated during cleanup, so existing local clones can
be ahead/diverged even with a clean working tree. Reset the local PX4 checkout to
`origin/px4xplane-sitl` once, then use `git pull --ff-only` afterward.
