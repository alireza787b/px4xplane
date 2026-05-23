# Report v47 - QuadTailsitter qtail12 FW Orbit and Pitot Density

Date: 2026-05-23  
Package target: `v3.4.35`

## Inputs

- `/home/alireza/qtail12.zip`
- PX4 ULog: `07_34_41.ulg`
- X-Plane log: `Log.txt`
- TruthCapture run: `20260523-073119Z`

The zip is valid. TruthCapture recorded `61,908` frames over `773.5 s` of
sim time with zero dropped rows, zero sim-time resets, about `80.1 Hz` mean
callback rate, and `50 ms` worst frame period.

## Startup Warnings

The initial `Airspeed selector module down` and `ekf2 missing data` warnings
occur before the simulator stream is fully settled. In the ULog, PX4 later
became ready for takeoff and `airspeed_validated.airspeed_source` was sensor
source `1` for the controlled flight. This is a transient startup ordering
warning, not evidence that qtail12 used open-loop transition.

The first front-transition attempt around `67.8 s` was commanded while the
vehicle was about `37.5 m` above home, below the current `VT_FW_MIN_ALT=40 m`.
PX4 correctly treated that as a fixed-wing system failure/quad-chute condition.
The later transition at about `110.7-119.4 s` was the useful transition sample.

## IAS, CAS, TAS

PX4 does not control directly from a raw X-Plane TAS field when a pitot sensor
is configured. The bridge sends `HIL_SENSOR.diff_pressure` in hPa. PX4 converts
that to `differential_pressure` in Pa, then the sensors pipeline publishes
`airspeed` and `airspeed_validated` with indicated/calibrated and true airspeed.

For conventional aircraft, `airspeedSource=xplane_indicated` converts X-Plane
IAS to equivalent dynamic pressure using sea-level density. For the tailsitter,
`airspeedSource=body_axis` projects local air-relative velocity onto
`pitotAxisBody=-Z`. That projection is a local true-flow component, so v3.4.35
now converts it to differential pressure using X-Plane local
pressure/temperature density. PX4 then derives calibrated and true airspeed
through its normal path.

qtail12 did use the modeled pitot: `airspeed_validated.airspeed_source=1`, and
front transition completed when calibrated airspeed reached about `20 m/s`.

## FW Orbit and RTL Findings

The aircraft was not airspeedless and it was not failing to bank. In fixed-wing
coordinates, actual bank followed the requested direction, but guidance was too
aggressive for the achieved speed and radius.

Key qtail12 measurements:

- FW hold: groundspeed mean `32.2 m/s`, calibrated airspeed median
  `30.8 m/s`, transformed bank median `35.2 deg`.
- FW RTL: groundspeed mean `37.5 m/s`, calibrated airspeed median
  `35.5 m/s`, transformed bank median `34.3 deg`.
- Lateral acceleration setpoint repeatedly saturated at `+/-6.87 m/s^2`,
  which corresponds to the configured `FW_R_LIM=35 deg`.
- The FW hold fit was about `328 m` radius with large residuals; RTL fit was
  about `664 m` with very large residuals, so the vehicle was not cleanly
  settling into the requested path.
- The main control allocator instance reported achieved torque. This does not
  support motor cant as the first fix. Cant remains a possible future A/B test,
  but only if a later log shows roll/yaw torque cannot be achieved.

The practical root cause is the combination of speed, a `300 m` loiter/RTL
radius, and an NPFG tune that requests abrupt roll-limit corrections when the
vehicle is already outside the desired path.

## Changes Made

Bridge:

- `body_axis` pitot dynamic pressure now uses local X-Plane pressure and
  temperature density.
- `xplane_indicated` remains sea-level-density equivalent, matching IAS/EAS
  behavior.

PX4 QuadTailsitter params:

- `FW_AIRSPD_TRIM: 24 -> 22`
- `FW_AIRSPD_MAX: 35 -> 34`
- `FW_THR_TRIM: 0.12 -> 0.08`
- `FW_THR_MIN: 0.03 -> 0.00`
- `FW_THR_MAX: 0.45 -> 0.35`
- `VT_ARSP_BLEND: 14 -> 13`
- `VT_ARSP_TRANS: 20 -> 18`
- `NPFG_PERIOD: 14 -> 22`
- `NPFG_DAMPING: 0.75 -> 0.85`
- `NPFG_ROLL_TC: 0.7 -> 1.2`
- `NAV_LOITER_RAD: 300 -> 500`
- `RTL_LOITER_RAD: 300 -> 500`

No motor cant was added in this slice. Adding cant must be matched between the
X-Plane ACF motor axes and PX4 `CA_ROTOR*_AX/AY/AZ`; doing only one side would
make the actuator model inconsistent.

## Next Test

Use the packaged QuadTailsitter aircraft and pull the PX4 PR branch containing
`5021_xplane_qtailsitter`. Force a clean PX4 parameter load before judging:
`make distclean` is safest.

Test sequence:

1. Take off and hold in MC mode.
2. Do one Go-To at `3 m/s`, then one at `4-5 m/s`.
3. Climb above `80 m` AGL before transition.
4. Forward transition into open space.
5. Hold straight for `10-15 s`.
6. Test FW loiter/orbit with at least `500 m` radius.
7. Try RTL only after FW loiter is stable.
8. Back-transition manually if RTL path following becomes unstable.

Acceptance targets:

- `airspeed_validated.airspeed_source=1`.
- Transition completes on calibrated airspeed near `18 m/s`.
- FW speed stays closer to `22-30 m/s` instead of accelerating into the high
  `30s`.
- Loiter/RTL does not spend long periods at the `35 deg` roll limit.
- No quad-chute unless transition is commanded below the configured minimum
  altitude.
