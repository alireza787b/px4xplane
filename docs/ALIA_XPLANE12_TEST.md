# Alia250 X-Plane 12 Test Card

## Setup

Use the matching lift-plus-cruise model: four hover motors, one pusher,
ailerons, elevator and rudder. The handling reference used a loaded mass of
2857.22 kg. PX4 `WEIGHT_BASE` and `WEIGHT_GROSS` do not set X-Plane loading.

1. Load the model in X-Plane 12 and select **Alia250** in the plugin.
   The configuration section and saved `config_name` are `Alia250`.
2. Start the normal launcher:

   ```bash
   px4xplane --airframe xplane_alia250
   ```

3. Select **Connect to SITL** when PX4 is waiting. Confirm QGC receives
   telemetry, the estimator is ready, and `SYS_AUTOSTART=5020`.

The normal launcher uses the maintained `PX4-Autopilot-Me` fork, branch
`px4xplane-airframes`. On first adoption of changed defaults, explicitly reload
only this airframe's saved parameters:

```bash
px4xplane --airframe xplane_alia250 --reset-params
```

Ordinary launches preserve saved tuning. See the
[launcher reference](guided-launcher.md#saved-parameters) for state and backups.

## Selected Parameters

The canonical reference is
[`5020_xplane_alia250`](../config/px4_params/5020_xplane_alia250).
These eight settings preserve the accepted handling tune:

| Parameter | Value |
| --- | ---: |
| WV_YRATE_MAX | 10 |
| FW_P_TC | 1.0 |
| VT_B_DEC_I | 0.08 |
| VT_B_TRANS_DUR | 40 |
| VT_ARSP_TRANS | 48 |
| FW_RR_FF | 1.5 |
| CA_SV_CS0_TRIM | -0.3333333333 |
| CA_SV_CS1_TRIM | -0.3333333333 |

`MPC_AUTO_NUDGING=0` preserves autonomous behavior when replacing the obsolete,
ignored `MPC_LAND_RC_HELP` assignment.

Keep `PWM_MAIN_REV=112`, the existing output order, and the elevator/rudder
trims unchanged. Aileron endpoints remain `[-20 10]` degrees; elevator and
rudder endpoints remain `[-15 20]`. The two aileron trims calibrate common
neutral without changing those endpoints. Lift-prop auto braking remains
disabled with an empty `autoPropBrakes` setting.

PX4 executes the airframe in its checkout under
`ROMFS/px4fmu_common/init.d-posix/airframes/`; packaged reference copies are
not loaded directly from the plugin directory.

## Flight Check

1. Verify actual loading, CG, weather, QGC reception and the commanded takeoff
   altitude before arming.
2. Take off and settle in hover at the planned altitude.
3. Use a long straight segment for front transition. Check airspeed, height
   and the first fixed-wing altitude capture before turning.
4. Check settled fixed-wing Hold and a controlled return/back-transition.
5. Verify descent, touchdown and automatic disarm. Wait 15 seconds after
   disarm, then enter `shutdown` in the PX4 console.

## Developer Checks

- The accepted November 8 flight used source `95a0c2348232`. Integration uses
  upstream base `0e0f3ce8a2`; flight validation of that newer core is pending.
- The accepted flight completed front transition, fixed-wing Hold, return,
  back-transition and landing. It did not establish the requested 120-second
  straight segment, a complete circle, or a payload/wind/FPS envelope.
- Preserve the selected transition values during metadata reconciliation.
  `VT_ARSP_TRANS=48` and `VT_B_TRANS_DUR=40` exceed generic metadata limits;
  clamping them would change the flown tune.
- Compare effective parameters, ULog tracking, actuator saturation and
  estimator faults. TruthCapture is optional development evidence, not a
  normal launcher dependency.
