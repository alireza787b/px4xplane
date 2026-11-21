# Cessna 172 X-Plane 12 Test Card

## Setup

Use the stock analog Cessna 172 SP with mirrored left/right elevators,
separate nosewheel steering and landing flaps. The recorded comparison used
1044.2 kg total mass and a +0.148 m longitudinal CG offset. Check the actual
Weight and Balance settings; PX4 `WEIGHT_*` values do not set model loading.

1. Load the aircraft on a paved runway and select **Cessna 172** in the plugin.
   The configuration section and saved `config_name` are `Cessna172`.
2. Start the beta/developer launcher:

   ```bash
   px4xplane --airframe xplane_cessna172
   ```

3. Select **Connect to SITL** when PX4 is waiting. Confirm QGC receives
   telemetry, the estimator is ready, and `SYS_AUTOSTART=5001`.
4. Check actual braking after connection and release the parking brake
   before starting the runway takeoff.

The beta/developer launcher uses the maintained `PX4-Autopilot-Me` fork, branch
`px4xplane-airframes`. To adopt changed defaults explicitly, use:

```bash
px4xplane --airframe xplane_cessna172 --reset-params
```

Ordinary launches preserve saved tuning. See the
[launcher reference](guided-launcher.md#saved-parameters) for state and backups.

## Selected Parameters And Mapping

The canonical reference is
[`5001_xplane_cessna172`](../config/px4_params/5001_xplane_cessna172).
The selected comparison settings are:

| Parameter | Value |
| --- | ---: |
| FW_P_LIM_MIN | -5.0 |
| FW_WR_P | 1.0 |

Keep `FW_AIRSPD_TRIM=40`, `FW_THR_MIN=0.10`, `FW_THR_SLEW_MAX=0.25`,
`FW_FLAPS_TO_SCL=0.33` and `CA_SV_FLAP_SLEW=0.08`.
`PWM_MAIN_REV=7`; outputs 6, 7 and 8 remain functions 440, 205 and 206
for nosewheel steering and the two flaps.

Both elevator halves use the existing channel 2:

```ini
channel2 = sim/flightmodel2/wing/elevator1_deg, floatArray, [8 9], [-20 +20]
```

Indices are space-separated. Preserve the sign, range and 0.04-second
smoothing on channels 0, 1, 2 and 4. Keep nosewheel steering separate from
rudder, and retain the existing throttle, aileron and flap mappings.
Airspeed remains `xplane_indicated`.

PX4 executes its checkout's airframe under
`ROMFS/px4fmu_common/init.d-posix/airframes/`, not the packaged reference
inside the plugin directory.

## Flight Check

1. Confirm loading, weather, runway alignment and QGC telemetry before arming.
2. Verify both elevators respond together and runway steering acts through
   the dedicated nosewheel channel.
3. Check climbout, straight cruise, descent, flare, touchdown and rollout
   without changing gains in flight.
4. Use the mission's actual height, radius, speed and final geometry when
   comparing tracking; mission items can override profile defaults.
5. After landing and automatic disarm, wait 15 seconds and enter `shutdown`
   in the PX4 console.

## Developer Checks

- The November 17 parked integration check connected, stayed disarmed and
  stopped correctly. It did not validate paired elevator response in flight.
  The preceding flight moved only the left elevator; the corrected mapping
  still requires a flight comparison with the same mission/loading/gains.
- The two selected settings were flown on source `95a0c2348232`.
  Integration uses upstream base `0e0f3ce8a2`; flight validation of that
  newer core is also pending.
- Keep the mapping comparison separate from approach-geometry qualification.
  The earlier short final landed long; the proposed 150 m approach and roughly
  3 km stabilized final have not been qualified by those returns.
- Do not add an airspeed correction or retune TECS to hide the model's
  instrument-IAS behavior. Compare both physical elevators, pitch/height
  tracking, descent, flare and rollout before closing the flight check.
- Retain source/model identity, effective parameters and ULog results.
  TruthCapture is optional development evidence.
