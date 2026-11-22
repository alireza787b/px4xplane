# QuadTailsitter X-Plane 12 Test Card

## Setup

Use the matching 2 kg, 0.6 m-span, 6S model with four canted 10x8 rotors and
no aerodynamic control surfaces. The canonical profile is paired with this
model and its original CG, not the earlier 5 kg dynamics.

1. Load the matching model in X-Plane 12 and select **QuadTailsitter** in the
   plugin. The configuration section and saved `config_name` remain
   `QuadTailsitter`.
2. Start the beta/developer launcher:

   ```bash
   px4xplane --airframe xplane_qtailsitter
   ```

3. Select **Connect to SITL** when PX4 is waiting. Confirm QGC receives
   telemetry, the estimator is ready, and `SYS_AUTOSTART=5021`.

The plugin's **Nose / FPV** view follows the fixed camera mount and aircraft
attitude; it is not a horizon-stabilized gimbal. **Belly / Down** looks forward
in hover and down in fixed-wing flight. For a chase view, release the plugin
camera and select X-Plane's own external view. Disable another camera plugin's
control when isolating conflicting views.

The beta/developer launcher uses the maintained `PX4-Autopilot-Me` fork, branch
`px4xplane-airframes`. To adopt changed defaults explicitly, use:

```bash
px4xplane --airframe xplane_qtailsitter --reset-params
```

Ordinary launches preserve saved tuning. See the
[launcher reference](guided-launcher.md#saved-parameters) for state and backups.

## Parameter Reference

The canonical reference is
[`5021_xplane_qtailsitter`](../config/px4_params/5021_xplane_qtailsitter).
It contains **209 defaults**: the frozen 211-value profile with the four
obsolete `FD_FAIL_R`, `FD_FAIL_P`, `FD_FAIL_R_TTRI` and `FD_FAIL_P_TTRI`
assignments replaced by `FD_TILT_ERROR=85` and `FD_TILT_ERROR_T=1`.
All other frozen values are preserved.

Selected cross-checks, not a replacement for the complete profile:

| Parameter | Value |
| --- | ---: |
| MPC_THR_HOVER | 0.26 |
| MC_ROLLRATE_P | 0.06 |
| MC_PITCHRATE_P | 0.16 |
| FW_RR_P | 0.26 |
| FW_T_I_GAIN_PIT | 0.10 |
| FW_THR_TRIM | 0.18 |
| FW_AIRSPD_STALL | 17 |
| FW_AIRSPD_MIN | 21 |
| FW_AIRSPD_TRIM | 26 |
| FW_AIRSPD_MAX | 34 |
| VT_ARSP_TRANS | 25 |
| VT_B_TRANS_DUR | 15 |
| FD_TILT_ERROR | 85 |
| FD_TILT_ERROR_T | 1 |

Keep the complete rotor geometry, effectiveness, MC/FW gains and transition
settings together. The four motor channels address engine indices 0-3.
Airspeed remains `body_axis` with `pitotAxisBody=-Z`; actuator smoothing
remains zero.

PX4 executes its checkout's airframe under
`ROMFS/px4fmu_common/init.d-posix/airframes/`, not the packaged reference
inside the plugin directory.

## Flight Check

1. Confirm model loading, original CG, QGC telemetry and effective parameters.
2. Set the planned takeoff and transition heights in the mission. The profile's
   `MIS_TAKEOFF_ALT=1.5` is not suitable transition clearance.
3. Begin with hover, a modest movement/stop and a controlled Land after an
   installation change.
4. At the planned transition height, check front transition, settled
   fixed-wing flight, back-transition and return.
5. Judge body rates, quaternions and height as well as attitude; Euler roll
   alone is misleading near vertical. Stop for a growing departure.
6. Verify controlled descent, touchdown and automatic disarm. Wait 15 seconds,
   then enter `shutdown` in the PX4 console.

## Developer Checks

- The R16 handling reference used logging child `58794523a151` of upstream
  `649929706fb3`. Integration uses upstream base `0e0f3ce8a2`; the newer
  core and failure-detector migration passed an owner baseline flight on
  November 22 with maintained commit `ecff79850f`: forward/back-transition,
  RTL, touchdown and automatic disarm, at the original 2 kg/CG configuration.
  This is not a payload, CG, wind or hardware-envelope validation.
- Upstream commits `7f83c28400` and `fa877566a4` replace the old roll/pitch
  failure thresholds with tilt-error settings. Do not restore the removed
  four parameters to obtain the historical count of 211.
- The accepted flight used mission takeoff at 50 m and about 25 seconds of
  fixed-wing flight before RTL. It did not validate the prescribed 180 m
  takeoff/60-second fixed-wing scenario or a payload/CG/wind/FPS envelope.
- Preserve `FW_R_TC=1.5` during metadata review; clamping it changes the
  tune. Remaining fixed-wing oscillation and back-transition height gain
  belong in the flight comparison.
- Retain source/model identity, effective parameters and ULog results.
  TruthCapture is optional development evidence.
