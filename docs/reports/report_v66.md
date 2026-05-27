# Report v66 - Cessna 172 Runway Steering and Flare Recovery

Date: 2026-05-27

Package target: `v3.4.54`

Evidence reviewed:

- `/home/alireza/cessna1.zip`
- ULog: `/tmp/cessna1/07_24_12.ulg`
- X-Plane log: `/tmp/cessna1/Log.txt`
- TruthCapture: `/tmp/cessna1/20260527-071813Z`

## Summary

`cessna1` is a useful first Cessna validation. The airborne mission and final
approach were broadly good, but the run exposed two structural setup issues and
one landing-energy tune issue:

- PX4 generated wheel-steering commands, but the Cessna airframe did not map
  `Landing_Gear_Wheel` to an output and the bridge did not expose a Cessna
  channel for the nosewheel.
- The high accelerometer-bias preflight warning was not a reason to raise
  `EKF2_ABL_LIM` beyond PX4's declared range. The log committed a stable
  simulated accelerometer calibration at disarm.
- The first touchdown was too fast and bounced, so the no-flap landing baseline
  needs lower landing airspeed and earlier flare.

## Evidence

TruthCapture recorded `92,435` rows with `0` dropped rows, `0` sim-time resets,
and about `92 Hz` mean callback rate. This does not point to a low-FPS sensor
pipeline failure in this run.

X-Plane loaded the Laminar Cessna 172 and px4xplane `v3.4.53`. The bridge was
manually switched to Cessna before PX4 connected:

```text
Config Name: Cessna 172 (Cessna172)
No configuration for channel 5 in Cessna172
```

The ULog loaded `SYS_AUTOSTART=5001` and showed:

```text
EKF2_ABL_LIM = 2.0
CAL_ACC0_*OFF = 0
PWM_MAIN_FUNC6 = 0
FW_W_EN = 1
```

The `landing_gear_wheel` topic was present and commanded about `-0.80..0.78`
normalized wheel steering, but with `PWM_MAIN_FUNC6=0` those commands were not
sent to the bridge. PX4 source defines `Landing_Gear_Wheel` as output function
`440` in `src/lib/mixer_module/output_functions.yaml`.

At disarm, PX4 committed:

```text
ACC 0 offset committed: [0.000 0.000 0.000]->[-0.018 -0.139 -0.797]
```

The same ULog had `estimator_sensor_bias.accel_bias[2]` near `-0.796 m/s^2`.
PX4 metadata caps `EKF2_ABL_LIM` at `0.8`, and commander uses a fraction of
that limit for the preflight check. The correct next step is to seed the
simulated Cessna accelerometer calibration, not to keep an out-of-range limit.

TruthCapture touchdown evidence:

```text
first touchdown: 46.9 m/s groundspeed, bounced
final touchdown: 34.7 m/s groundspeed
```

The first touchdown was far above the intended Cessna approach/touchdown range.
Textron lists the current Skyhawk stall speed as `48 KCAS`, max cruise as
`124 KTAS`, and max climb rate as `730 fpm`, so a no-flap SITL landing target
around `33 m/s` is a conservative next baseline rather than an aggressive short
field setting.

## v3.4.54 Changes

Bridge config:

- Make `Cessna172` the active packaged airframe for this Cessna test slice.
- Split rudder and nosewheel steering:
  - `channel4`: rudder only
  - `channel5`: `sim/flightmodel2/gear/tire_steer_command_deg[0]`

PX4 Cessna airframe:

- Add `PWM_MAIN_FUNC6=440` for PX4 `Landing_Gear_Wheel`.
- Add `CAL_ACC0_PRIO=50` and `CAL_ACC0_XOFF/YOFF/ZOFF =
  -0.018/-0.139/-0.797`.
- Keep `EKF2_ABL_LIM=0.8` instead of adopting the temporary `2.0` value from
  the test run.
- Smooth roll response:
  - `FW_R_RMAX: 25 -> 20`
  - `FW_R_TC: 0.7 -> 0.9`
  - `FW_PN_R_SLEW_MAX: 50 -> 35`
- Improve takeoff clearance:
  - `MIS_TAKEOFF_ALT: 100 -> 120`
- Improve no-flap landing:
  - `FW_LND_AIRSPD: 36 -> 33`
  - `FW_LND_FLALT: 5 -> 6`
  - `FW_LND_FL_TIME: 1 -> 2`
  - `FW_LND_FL_PMIN: 4 -> 5`
  - `FW_LND_FL_PMAX: 18 -> 20`
  - `FW_LND_TD_TIME: -1 -> 3`
  - `LNDFW_AIRSPD_MAX: 30 -> 35`

Config editor:

- The left airframe list is no longer collapsed.
- The middle editor area now uses collapsible groups for global runtime fields,
  airframe setup fields, actuator mappings, and camera views.
- The selected airframe is shown as an editing banner; `config_name` remains in
  global runtime fields because it is the actual runtime source of truth.
- Empty-state copy now explicitly says that `config.ini` is the runtime file and
  JSON is optional schema metadata only.

## What Was Not Changed

- No bridge sign, IMU axis, GPS velocity, or airspeed source rewrite was made.
  `cessna1` does not show a fundamental sensor-contract failure.
- No flap modeling was added yet. This release keeps a no-flap Cessna baseline
  so the wheel steering and flare changes can be isolated.
- No broad TECS/NPFG retune was made because the accepted airborne mission path
  should not be disturbed until the structural runway steering issue is fixed.

## References

- PX4 fixed-wing takeoff documents runway takeoff, `FW_W_EN`, wheel-controller
  tuning, and the clearance altitude behavior:
  <https://docs.px4.io/v1.14/en/flight_modes_fw/takeoff>
- PX4 fixed-wing mission landing documents the loiter-to-altitude approach,
  flare, `FW_LND_FL_TIME`, `FW_LND_FLALT`, and `FW_LND_TD_TIME`:
  <https://docs.px4.io/v1.15/en/flight_modes_fw/mission.html#mission-landing>
- Textron Cessna Skyhawk specifications:
  <https://cessna.txtav.com/piston/cessna-skyhawk>

## Next Test

Use `v3.4.54` with a clean PX4 parameter state:

```bash
make px4_sitl_default distclean
make px4_sitl_default xplane_cessna172
```

Before judging the flight:

```bash
python3 tools/check_px4_airframe_params.py /path/to/cessna.ulg \
  config/px4_params/5001_xplane_cessna172
```

Expected sanity items:

```text
Config Name: Cessna 172 (Cessna172)
Parsed channel 5
PWM_MAIN_FUNC6 = 440
CAL_ACC0_ZOFF = -0.797
EKF2_ABL_LIM = 0.8
FW_LND_AIRSPD = 33
```

Mission recommendation:

1. Start aligned on a paved runway, brake released.
2. Use runway takeoff to `120 m` AGL.
3. Put the first outbound waypoint at least `900 m` from the runway and roughly
   along runway heading.
4. Use a broad `600 m` loiter or gentle turn.
5. Use a fixed-wing landing pattern to the same runway.
