# Cessna 172 X-Plane 12 Test Card

Use this card for the next validation of the refreshed Cessna 172 runway
mission baseline.

## Package And PX4 State

1. Install the px4xplane package. This Cessna test package intentionally ships
   with:

   ```ini
   config_name = Cessna172
   ```

   After the Cessna phase closes, normal demo packages can return to the Alia
   default.

2. Run PX4 with the validation launcher and select `xplane_cessna172`:

   ```bash
   px4xplane --validation --reset-config
   ```

   The printed validation stack should show official PX4 plus the pending
   EKF-GSF, Standard VTOL, and TECS PR guards unless you are
   deliberately testing `--official`.

3. For a fully manual developer run after changing the airframe file, reset the
   PX4 SITL parameter store once and launch:

   ```bash
   make px4_sitl_default distclean
   make px4_sitl_default xplane_cessna172
   ```

4. Use the X-Plane Cessna 172 on a paved runway. Align the aircraft on the
   runway heading before arming. Release parking brake before mission start.

5. Start XPlaneTruthCapture before connecting PX4.

## Mission Shape

Use a simple, conservative fixed-wing mission:

1. Runway takeoff to `150 m` AGL.
2. One straight outbound waypoint at least `900 m` from the runway.
3. One gentle turn waypoint or loiter with radius around `600 m`.
4. Fixed-wing landing pattern back to the same runway. This package uses
   landing flaps, so give the aircraft enough final distance to stabilize
   before flare.

Avoid judging the first test with a very tight loiter or short waypoint spacing.
The Cessna is a manned fixed-wing aircraft, not an agile UAV.

## Expected Values

PX4 airframe sanity values:

```text
SYS_AUTOSTART = 5001
EKF2_MULTI_IMU = 1
SENS_IMU_MODE = 0
CAL_ACC0_PRIO = 50
CAL_ACC0_XOFF = 0.0
CAL_ACC0_YOFF = 0.0
CAL_ACC0_ZOFF = 0.0
EKF2_ABL_LIM = 0.8
EKF2_ACC_B_NOISE = 0.001
FW_USE_AIRSPD = 1
ASPD_DO_CHECKS = 1
ASPD_FALLBACK = 1
SYS_HAS_NUM_ASPD = 0
FW_AIRSPD_MIN = 28
FW_AIRSPD_TRIM = 40
FW_AIRSPD_MAX = 65
FW_AIRSPD_FLP_SC = 0.85
FW_TKO_AIRSPD = 40
FW_TKO_PITCH_MIN = 10
RWTO_TKOFF = 1
RWTO_ROT_AIRSPD = 32
FW_FLAPS_TO_SCL = 0.33
CA_SV_FLAP_SLEW = 0.08
FW_W_EN = 1
PWM_MAIN_FUNC6 = 440
PWM_MAIN_FUNC7 = 205
PWM_MAIN_FUNC8 = 206
FW_RR_FF = 3.2
FW_RR_P = 0.42
FW_RR_D = 0.08
FW_R_RMAX = 8
FW_R_TC = 1.5
FW_R_LIM = 15
FW_LND_AIRSPD = 32.5
FW_LND_EARLYCFG = 1
FW_LND_ANG = 5
FW_LND_FLALT = 8
FW_LND_FL_TIME = 4
FW_LND_FL_PMIN = 9
FW_LND_TD_TIME = -1
FW_FLAPS_LND_SCL = 0.85
FW_WR_P = 0.8
FW_WR_I = 0.12
FW_WR_FF = 0.35
FW_W_RMAX = 30
LNDFW_AIRSPD_MAX = 12
LNDFW_VEL_XY_MAX = 5
LNDFW_TRIG_TIME = 2
NAV_ACC_RAD = 180
NAV_LOITER_RAD = 600
```

Keep the accelerometer offsets neutral. The bridge already publishes
attitude-consistent HIL accelerometer data; nonzero offsets can corrupt
`sensor_combined` and trigger EKF2 accelerometer-bias checks while parked.

Bridge sanity values:

```text
Config Name: Cessna172
Active Airframe: Cessna172
Airspeed source: xplane_indicated
Camera Views: Forward, Down, Chase
Actuator command smoothing enabled (tau 0.040s, channels 0,1,2,4)
Parsed channel 5
Parsed channel 6
Parsed channel 7
```

## Acceptance Criteria

- Runway takeoff accelerates, rotates, and climbs without wingstrike or
  immediate heading divergence. If it leaves centerline, check that the ULog has
  `PWM_MAIN_FUNC6 = 440` and X-Plane `Log.txt` shows `Parsed channel 5`.
  Earlier Cessna logs that loaded an older package did not test the dedicated
  Cessna wheel channel.
- No in-flight EKF, baro stale, accelerometer bias, or airspeed selector
  warnings after sensors settle.
- TECS holds altitude without phugoid-like throttle/pitch hunting in straight
  cruise.
- NPFG turns are broad and coordinated; no repeated path overshoot around the
  `600 m` loiter.
- Mission landing follows the final approach path, slows before final, visibly
  deploys landing flaps, flares near the runway, and disarms after a controlled
  rollout at low ground speed.
- X-Plane `Log.txt` should not report replaced or missing Cessna flap
  datarefs. The current package writes `sim/cockpit2/controls/flap_handle_request_ratio`
  and the physical `sim/flightmodel/controls/wing1l_fla1def` /
  `sim/flightmodel/controls/wing1r_fla1def` datarefs.
- Takeoff flaps are intentional in this package (`FW_FLAPS_TO_SCL = 0.33`).
  They should move smoothly because `CA_SV_FLAP_SLEW = 0.08`; abrupt flap
  retraction usually means stale PX4 parameters or an aircraft-side override.

## Log Bundle

Return:

- PX4 ULog
- X-Plane `Log.txt`
- PX4 terminal log
- XPlaneTruthCapture folder or zip
- Notes on runway heading, wind, mission waypoint spacing, and any QGC warning

Run the static parameter check before interpreting flight behavior:

```bash
python3 tools/check_px4_airframe_params.py /path/to/cessna.ulg \
  config/px4_params/5001_xplane_cessna172
```

If the ULog does not contain the values above, reset PX4 SITL parameters and
rerun before tuning.

For the Cessna phase, do not skip the reset. `cessna4` contained saved
parameter overrides from manual tuning, including old roll and EKF values, so
the static parameter check is part of the test procedure, not an optional
cleanup step.
