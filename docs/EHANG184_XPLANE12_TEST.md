# EHang184 X-Plane 12 Test Card

## Setup

Use a four-motor model matching the EHang184 allocator and engine order.
The recorded handling reference used 245 kg with neutral recorded CG offsets
and calm conditions. Model loading and motor layout must match the reference,
not just the selected menu label.

1. Load the model in X-Plane 12 and select **Ehang 184** in the plugin.
   The configuration section and saved `config_name` remain `ehang184`.
2. Start the normal launcher:

   ```bash
   px4xplane --airframe xplane_ehang184
   ```

3. Select **Connect to SITL** when PX4 is waiting. Confirm QGC receives
   telemetry, the estimator is ready, and `SYS_AUTOSTART=5010`.

The normal launcher uses the maintained `PX4-Autopilot-Me` fork, branch
`px4xplane-airframes`. To adopt changed defaults explicitly, use:

```bash
px4xplane --airframe xplane_ehang184 --reset-params
```

Ordinary launches preserve saved tuning. See the
[launcher reference](guided-launcher.md#saved-parameters) for state and backups.

## Selected Parameters

The canonical reference is
[`5010_xplane_ehang184`](../config/px4_params/5010_xplane_ehang184).

| Parameter | Value |
| --- | ---: |
| MPC_THR_HOVER | 0.25 |
| MPC_YAWRAUTO_MAX | 10 |
| MPC_YAWRAUTO_ACC | 10 |

`MPC_AUTO_NUDGING=0` preserves autonomous behavior when replacing the obsolete,
ignored `MPC_LAND_RC_HELP` assignment. PID gains, allocation and thrust curve
remain unchanged.

Plugin channels 0-3 address engine indices 0-3. Keep the allocator's motor
positions, rotation signs and output order paired with the loaded model.
PX4 body axes are +X forward, +Y right and +Z down; inherited motor-position
comments are not a substitute for checking that pairing.

PX4 executes its checkout's airframe under
`ROMFS/px4fmu_common/init.d-posix/airframes/`, not the packaged reference
inside the plugin directory.

## Flight Check

1. Verify actual loading, CG, weather and QGC reception before arming.
2. Take off to the planned height and allow at least 20 seconds of settled
   hover before evaluating position, height and yaw response.
3. Check a modest GoTo move and a settled stop. Verify the actual mode and
   setpoint before treating a command acknowledgement as tracking.
4. Check a controlled return, descent, touchdown and automatic disarm.
5. Wait 15 seconds after disarm, then enter `shutdown` in the PX4 console.

Mission takeoff height is a scenario choice, not part of the three-value tune.

## Developer Checks

- The frozen November 8 handling flights used source `95a0c2348232`.
  Integration uses upstream base `0e0f3ce8a2`; flight validation of that
  newer core is pending.
- Hover, GoTo response, short settled Orbit arcs, RTL and landing were
  demonstrated. A full settled Orbit and separate 50 m/30 m height holds
  remain unchecked.
- Compare hover-thrust estimation, yaw allocation residuals, lower motor
  limits and touchdown behavior. The repeat retained a landing-yaw transient;
  the freeze does not mean zero saturation or identical-input A/B results.
- Record source/model identity, effective parameters and ULog results for
  comparisons. TruthCapture is optional development evidence.
