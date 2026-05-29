# Report v77 - X-Plane GNSS Timing and Alia Compass Fault Recovery

Date: 2026-05-29

## Scope

Reviewed `/home/alireza/18_33_37.ulg` after the Alia run that showed startup
`no heading reference`, in-flight `Compass 0 fault`, and a quad-chute during
front transition.

## Findings

The startup `Preflight Fail: no heading reference` was transient. In the ULog,
EKF yaw alignment became true at about `1.08 s`, heading became observable at
about `1.13 s`, and PX4 reported `Ready for takeoff` at about `2.13 s`. That
warning is harmless if it clears before arming readiness.

The in-flight compass warning was not a raw magnetometer failure. The magnetic
field magnitude stayed stable near `0.488 G`, and the reconstructed NED field
from `sensor_mag` plus X-Plane ground-truth attitude stayed consistent through
transition.

The failure chain was:

1. X-Plane HIL GPS published `timestamp_sample=0`.
2. The current airframe defaults forced `SENS_GPS0_DELAY=0` and
   `SENS_GPS1_DELAY=0`, so PX4 could not back-date the GPS measurement.
3. During front transition, GNSS position innovations reached the PX4 gate and
   repeatedly rejected fusion.
4. EKF2 performed emergency yaw resets. PX4 then marked the magnetometer as
   faulty because mag fusion was active during the yaw rescue path.
5. After fixed-wing handoff the aircraft lost altitude rapidly and crossed
   `VT_FW_MIN_ALT=20 m`, triggering quad-chute.

## Changes

- PX4 PR airframes:
  - `5001_xplane_cessna172`
  - `5002_xplane_tb2`
  - `5010_xplane_ehang184`
  - `5020_xplane_alia250`
  - `5021_xplane_qtailsitter`
- Packaged px4xplane mirrors:
  - `config/px4_params/*`

All now set:

```text
SENS_GPS0_DELAY=110
SENS_GPS1_DELAY=110
```

This matches PX4's standard fallback delay for GPS drivers that do not provide
their own sample timestamp correction.

## Next Test

Run Alia again from a clean PX4 SITL parameter store and confirm:

- `SYS_AUTOSTART=5020`
- `SENS_IMU_MODE=0`
- `SENS_GPS0_DELAY=110`
- `SENS_GPS1_DELAY=110`
- no in-flight `Compass 0 fault`
- no front-transition quad-chute

If `Preflight Fail: no heading reference` appears only before `Ready for
takeoff`, treat it as normal EKF startup alignment. If it remains after `Ready
for takeoff`, stop and capture the log.
