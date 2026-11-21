# TB2 X-Plane 12 Test Card

## Setup

Use the matching TB2-style model with inverted A-tail, separate nosewheel
steering and left/right flaps. The reference describes a modeled main-wing
span of about 10.8 m and a loaded mass of about 418 kg.

1. Load the aircraft on a long paved runway and select **TB2** in the plugin.
   The configuration section and saved `config_name` are `TB2`.
2. Start the beta/developer launcher:

   ```bash
   px4xplane --airframe xplane_tb2
   ```

3. Select **Connect to SITL** when PX4 is waiting. Confirm QGC receives
   telemetry, the estimator is ready, and `SYS_AUTOSTART=5002`.
4. Confirm runway alignment and release the parking brake before takeoff.

The beta/developer launcher uses the maintained `PX4-Autopilot-Me` fork, branch
`px4xplane-airframes`. To adopt changed defaults explicitly, use:

```bash
px4xplane --airframe xplane_tb2 --reset-params
```

Ordinary launches preserve saved tuning. See the
[launcher reference](guided-launcher.md#saved-parameters) for state and backups.

## Parameter Reference

Keep the existing TB2 tune in
[`5002_xplane_tb2`](../config/px4_params/5002_xplane_tb2).
No Alia250, EHang184, Cessna172 or QuadTailsitter gains are transferred to TB2.

| Parameter | Value |
| --- | ---: |
| FW_RR_P | 0.34 |
| FW_RR_I | 0.08 |
| FW_RR_D | 0.05 |
| FW_RR_FF | 2.4 |
| FW_PR_P | 0.32 |
| FW_PR_I | 0.04 |
| FW_PR_D | 0.08 |
| FW_PR_FF | 2.4 |
| FW_AIRSPD_TRIM | 36 |
| FW_LND_AIRSPD | 28 |
| FW_THR_TRIM | 0.32 |
| COM_DISARM_LAND | 4 |

Keep `PWM_MAIN_REV=15` and output functions
`201,202,203,204,101,440,205,206` for outputs 1-8. Plugin channels 2/3
control the A-tail, channel 4 the engine, channel 5 the nosewheel, and channels
6/7 the flaps. Airspeed remains `xplane_indicated`; control-surface smoothing
remains 0.04 seconds on channels 0-3.

PX4 executes its checkout's airframe under
`ROMFS/px4fmu_common/init.d-posix/airframes/`, not the packaged reference
inside the plugin directory.

## Flight Check

1. Verify actual loading, CG, weather and QGC telemetry before arming.
2. Use a straight runway takeoff and a long outbound climb before turning.
3. Check cruise tracking and broad loiter capture at the mission's actual
   height, radius and airspeed.
4. Give the fixed-wing landing pattern enough final distance to stabilize.
   Resolve an infeasible approach by reviewing height and distance.
5. Check A-tail response, flap deployment, flare, steering and rollout.
6. Wait 15 seconds after automatic disarm, then enter `shutdown` in the
   PX4 console.

## Developer Checks

- No new TB2 tuning evidence was supplied for this integration; retain its
  existing gains, speeds and landing tune.
- Integration uses upstream base `0e0f3ce8a2`. Flight validation of that
  newer core with this unchanged tune is pending.
- Compare source/model identity, effective parameters, ULog tracking and
  actuator outputs. Confirm both A-tail halves, the dedicated wheel output
  and both flaps follow their commands. TruthCapture is optional development
  evidence.
