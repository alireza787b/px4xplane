# Report v67 - Cessna2 Steering Evidence and Flap Landing Recovery

Date: 2026-05-27

Package target: `v3.4.55`

Evidence reviewed:

- `/home/alireza/cessna2.zip`
- ULog: `/tmp/cessna2/12_20_26.ulg`
- X-Plane log: `/tmp/cessna2/Log.txt`
- TruthCapture: `/tmp/cessna2/20260527-121744Z`

## Summary

`cessna2` did not actually test the latest bridge package. PX4 loaded the
v3.4.54 Cessna airframe and produced nosewheel commands, but X-Plane loaded
px4xplane `v3.4.53`, which did not include the Cessna channel 5 mapping. That
explains the runway drift: PX4 commanded steering, but the installed plugin
package was stale.

The flight also proved that no-flap landing-energy tuning is not enough for the
Laminar Cessna in this workflow. The aircraft still touched down around `90 kt`
IAS, so v3.4.55 adds real landing-flap outputs and uses early landing
configuration before final.

## Evidence

TruthCapture timing was clean:

```text
rows: 61,264
dropped rows: 0
sim-time resets: 0
mean callback rate: about 88 Hz
```

The X-Plane log showed:

```text
px4xplane: Version: v3.4.53
Airframe selected: Cessna 172 (Cessna172)
No configuration for channel 5 in Cessna172
```

The ULog showed the PX4 side was ahead of the installed bridge:

```text
PWM_MAIN_FUNC6 = 440
FW_W_EN = 1
FW_WR_P = 0.5
FW_WR_I = 0.1
FW_WR_FF = 0.2
```

During ground takeoff, PX4 published wheel-steering output:

```text
landing_gear_wheel.normalized_wheel_setpoint: -0.003 .. 0.546
actuator_outputs.output[5]: 1500 .. 1774 us
```

During landing rollout, PX4 again commanded wheel:

```text
landing_gear_wheel.normalized_wheel_setpoint: about -0.35
actuator_outputs.output[5]: about 1315 .. 1333 us
```

So the problem was not "PX4 never steered." The problem was that the X-Plane
package under test had no active Cessna wheel channel.

TruthCapture still saw only small tire-steer motion:

```text
sim/flightmodel2/gear/tire_steer_command_deg: -0.9 .. 3.5 deg
```

That range is consistent with X-Plane's default/coupled behavior, not a
dedicated PX4 wheel-controller path.

Landing evidence:

```text
first touchdown: 46.8 m/s groundspeed, about 91.7 kt IAS
second/final contact: about 44.8 m/s groundspeed, about 90 kt IAS
```

The autoland slope and flare logic were not enough to dissipate energy without
flaps. Lowering `FW_LND_AIRSPD` alone would hide the symptom while still leaving
the aircraft physically clean and fast on approach.

## v3.4.55 Changes

Bridge config:

- Cessna channel 5 now writes both wheel-steering command paths:
  - `sim/flightmodel2/gear/tire_steer_command_deg[0]`
  - `sim/flightmodel/parts/tire_steer_cmd[0]`
- Cessna channel 5 steering range increases to `[-20 +20] deg`.
- Cessna channels 6/7 now drive left/right landing flaps.
- Flap ranges are symmetric `[-40 +40]` so PX4 neutral command `0` writes
  `0 deg` in X-Plane and positive flap command writes down-flap.

PX4 Cessna airframe:

- Keeps `PWM_MAIN_FUNC6=440` for `Landing_Gear_Wheel`.
- Adds `CA_SV_CS4/5` as left/right flaps and maps:
  - `PWM_MAIN_FUNC7=205`
  - `PWM_MAIN_FUNC8=206`
- Increases wheel-controller authority:
  - `FW_WR_P: 0.5 -> 0.8`
  - `FW_WR_I: 0.1 -> 0.12`
  - `FW_WR_FF: 0.2 -> 0.35`
- Raises mission takeoff altitude:
  - `MIS_TAKEOFF_ALT: 120 -> 150`
- Adds flap-based landing-energy management:
  - `FW_AIRSPD_MIN: 30 -> 28`
  - `FW_AIRSPD_FLP_SC: 1.0 -> 0.85`
  - `FW_LND_AIRSPD: 33 -> 31`
  - `FW_LND_EARLYCFG: 0 -> 1`
  - `FW_LND_FLALT: 6 -> 8`
  - `FW_LND_FL_TIME: 2 -> 4`
  - `FW_LND_TD_TIME: 3 -> 5`
  - `FW_FLAPS_LND_SCL: 1.0 -> 0.65`
  - `FW_THR_MIN: 0.10 -> 0.05`

Config editor:

- Restores a simple airframe sidebar: all airframes remain visible; the active
  one is just marked.
- Replaces the confusing index-mode/raw-index controls with:
  - scalar mappings: `scalar`
  - array mappings: `[ number list ]` with brackets outside the input
- Replaces raw range text with `[ min max ]` inputs with brackets outside the
  fields.
- Keeps JSON as schema/help metadata only. The runtime file remains
  `px4xplane/64/config.ini`.

XPlaneTruthCapture:

- Bumped to v0.1.8.
- Added capture coverage for:
  - `sim/flightmodel/parts/tire_steer_cmd`
  - `sim/operation/override/override_wheel_steer`

## What Was Not Changed

- No IMU, GPS, baro, or airspeed sign/axis rewrite was made.
- No Cessna ACF physics edit was made in this slice.
- No broad TECS/NPFG retune was made beyond landing-energy, wheel, and takeoff
  altitude changes. The airborne path in `cessna2` was acceptable enough that
  another broad tuning pass would be speculative.

## Next Test

Install the full v3.4.55 package and confirm in X-Plane `Log.txt`:

```text
px4xplane: Version: v3.4.55
Config Name: Cessna 172 (Cessna172)
Parsed channel 5
Parsed channel 6
Parsed channel 7
```

Reset PX4 params before judging behavior:

```bash
make px4_sitl_default distclean
make px4_sitl_default xplane_cessna172
```

Expected PX4 sanity:

```text
PWM_MAIN_FUNC6 = 440
PWM_MAIN_FUNC7 = 205
PWM_MAIN_FUNC8 = 206
FW_LND_EARLYCFG = 1
FW_FLAPS_LND_SCL = 0.65
FW_LND_AIRSPD = 31
MIS_TAKEOFF_ALT = 150
```

Acceptance focus:

- takeoff stays on runway centerline
- first turn starts after a safer climb margin
- approach slows before final
- touchdown is not around `90 kt`
- rollout stays controlled without manual brake/steering recovery

## References

- PX4 fixed-wing runway takeoff and wheel steering:
  <https://docs.px4.io/v1.14/en/flight_modes_fw/takeoff>
- PX4 fixed-wing mission landing and flare timing:
  <https://docs.px4.io/v1.15/en/flight_modes_fw/mission.html#mission-landing>
- PX4 actuator function definitions are in
  `src/lib/mixer_module/output_functions.yaml` in the PX4 source tree.
- X-Plane plugin/dataref reference entry point:
  <https://developer.x-plane.com/datarefs/>
