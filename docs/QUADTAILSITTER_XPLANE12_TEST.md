# QuadTailsitter X-Plane 12 Hover, Go-To, Orbit, and First Transition Workflow

This card is for the next controlled QuadTailsitter validation after the
`qtail20.zip` run. v3.4.44 keeps the `5 kg` 6S 3115-class aircraft target and
the v3.4.43 stationary sensor/contact fixes, then softens multicopter Go-To
speed, acceleration, jerk, and tilt limits. qtail20 proved the bridge sanity
gate but showed a `5 m/s` Go-To command can still produce a much faster actual
speed burst if PX4 uses old trajectory defaults.

Go-To is still a point-hold command. It will decelerate near the target instead
of blending separate target clicks as one continuous path.

## Setup

1. Install the px4xplane package and set `px4xplane/64/config.ini` to:
   - `config_name = QuadTailsitter`
2. Install the packaged `QuadTailsitter` aircraft folder into X-Plane.
3. Install the PX4 airframe file:
   - `5021_xplane_qtailsitter`
4. Run PX4 with:
   - `make px4_sitl_default xplane_qtailsitter`
   - `SYS_AUTOSTART` must be `5021`
5. Force a clean PX4 parameter load before the test. Do not use only `make
   clean` for this airframe after a parameter-file change.
   - preferred: choose `d` / `make distclean`, then rebuild/run
     `xplane_qtailsitter`
   - targeted: delete any `parameters.bson` and `parameters_backup.bson` files
     under the PX4 SITL run directory, then rebuild/run
   - PX4 shell fallback: `param reset_all`, then stop and restart PX4
6. Do not judge the test if the ULog values do not match the sanity-check list
   below. qtail20 looked like v3.4.43 in X-Plane, but PX4 still used older
   qtail acceleration/jerk and land-detector defaults.
   - After the run, use:
     `python3 tools/check_px4_airframe_params.py <ulog> config/px4_params/5021_xplane_qtailsitter --param SYS_HAS_NUM_ASPD --param MPC_THR_HOVER --param CA_ROTOR0_AX --param CA_ROTOR1_AX --param MPC_XY_CRUISE --param MPC_ACC_HOR --param MPC_JERK_AUTO --param VT_ARSP_TRANS --param FW_THR_TRIM`
7. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off and let PX4 reach QGC's commanded takeoff altitude.
2. Hold in multicopter mode for `15-20 s`.
3. Command one modest Go-To movement. The intended cruise is now `3.5 m/s`.
4. Command one longer Go-To and watch for actual speed overshoot.
5. At `20 m` AGL or higher, command one `40-50 m` multicopter Orbit.
   - If QGC exposes Orbit yaw behavior, use hold initial heading or tangent
     heading for this test. If it does not, use the default center-facing Orbit
     and keep the radius at least `40 m`.
6. For the next package, stop here unless MC Go-To, RTL, and landing are clean.
   If actual horizontal speed stays below about `6 m/s` and attitude tracking
   is calm, climb high and attempt one straight forward transition.
7. RTL or Land and wait `10-15 s` after disarm before stopping PX4.

Abort transition immediately if uncontrolled descent starts, roll/pitch diverge
after transition should have stabilized, motors sit at min/max for more than a
brief recovery pulse, an attitude warning appears, or the vehicle travels out of
the safe test area.

QGC heading/attitude can look strange during tailsitter pitch-over near
vertical attitude. Judge transition state, local position, altitude, and the ULog
quaternion/attitude topics rather than QGC's small vehicle icon alone.

Go-To is a point-hold command. A smooth deceleration near the target is normal.
For continuous path-following, use a mission/path workflow or send the next
target before the aircraft reaches the current point.

## Parameter Sanity Check

Before judging the run, confirm these defaults in the ULog:

- `SYS_AUTOSTART=5021`
- `MAV_TYPE=20`
- `CA_AIRFRAME=4`
- `CA_SV_CS_COUNT=0`
- `CA_ROTOR0_PX=0.22`, `CA_ROTOR0_PY=0.43`
- `CA_ROTOR1_PX=-0.22`, `CA_ROTOR1_PY=-0.43`
- `CA_ROTOR2_PX=0.22`, `CA_ROTOR2_PY=-0.43`
- `CA_ROTOR3_PX=-0.22`, `CA_ROTOR3_PY=0.43`
- `CA_ROTOR0_AX=-0.078`, `CA_ROTOR0_AY=0.040`, `CA_ROTOR0_AZ=-0.996`
- `CA_ROTOR1_AX=0.078`, `CA_ROTOR1_AY=-0.040`, `CA_ROTOR1_AZ=-0.996`
- `CA_ROTOR2_AX=-0.078`, `CA_ROTOR2_AY=-0.040`, `CA_ROTOR2_AZ=-0.996`
- `CA_ROTOR3_AX=0.078`, `CA_ROTOR3_AY=0.040`, `CA_ROTOR3_AZ=-0.996`
- `CA_ROTOR0_CT=2.0`, `CA_ROTOR1_CT=2.0`
- `CA_ROTOR2_CT=2.0`, `CA_ROTOR3_CT=2.0`
- `CA_ROTOR0_KM=0.04`, `CA_ROTOR1_KM=0.04`
- `CA_ROTOR2_KM=-0.04`, `CA_ROTOR3_KM=-0.04`
- `MC_AIRMODE=2`
- `MC_ROLL_P=0.9`
- `MC_PITCH_P=0.50`
- `MC_YAW_P=0.80`
- `MC_YAW_WEIGHT=0.35`
- `MC_YAWRATE_P=0.20`
- `MC_YAWRATE_I=0.015`
- `MC_YAWRATE_D=0.025`
- `MC_YAWRATE_K=1.20`
- `MC_YAWRATE_MAX=65`
- `MC_ROLLRATE_P=0.10`
- `MC_PITCHRATE_P=0.10`
- `MC_ROLLRATE_D=0.0008`
- `MC_PITCHRATE_D=0.0010`
- `MC_ROLLRATE_K=1.00`
- `MC_PITCHRATE_K=0.85`
- `MC_ROLLRATE_MAX=80`
- `MC_PITCHRATE_MAX=80`
- `MPC_THR_HOVER=0.22`
- `MPC_THR_MIN=0.08`
- `MPC_THR_XY_MARG=0.4`
- `MPC_USE_HTE=0`
- `MPC_XY_P=0.18`
- `MPC_Z_P=1.00`
- `MPC_Z_VEL_P_ACC=2.0`
- `MPC_Z_VEL_I_ACC=0.5`
- `MPC_Z_VEL_D_ACC=0.5`
- `MPC_XY_CRUISE=3.5`
- `MPC_XY_VEL_MAX=4.0`
- `MPC_VEL_MANUAL=4.0`
- `MPC_XY_ERR_MAX=10`
- `MPC_ACC_HOR=0.7`
- `MPC_ACC_HOR_MAX=1.0`
- `MPC_JERK_AUTO=0.7`
- `MPC_JERK_MAX=1.2`
- `MIS_TAKEOFF_ALT=1.5`
- `MPC_Z_V_AUTO_UP=1.0`
- `MPC_Z_VEL_MAX_UP=1.2`
- `MPC_ACC_UP_MAX=1.6`
- `MPC_TKO_SPEED=1.0`
- `MPC_TKO_RAMP_T=2.5`
- `MPC_MAN_TILT_MAX=30.0`
- `MPC_TILTMAX_AIR=30.0`
- `MPC_YAW_MODE=0`
- `MPC_YAWRAUTO_MAX=35`
- `MPC_YAWRAUTO_ACC=12`
- `MIS_YAW_ERR=30`
- `MIS_YAW_TMT=5`
- `MC_ORBIT_YAW_MOD=1`
- `MPC_Z_V_AUTO_DN=1.4`
- `MPC_Z_VEL_MAX_DN=1.4`
- `MPC_LAND_SPEED=0.9`
- `MPC_LAND_CRWL=0.36`
- `LNDMC_Z_VEL_MAX=0.25`
- `FW_USE_AIRSPD=1`
- `ASPD_DO_CHECKS=1`
- `SYS_HAS_NUM_ASPD=0`
- `VT_ARSP_BLEND=18`
- `VT_ARSP_TRANS=24`
- `FW_AIRSPD_MIN=21`
- `FW_AIRSPD_TRIM=28`
- `FW_AIRSPD_MAX=40`
- `FW_THR_TRIM=0.16`
- `FW_THR_MAX=0.40`
- `FW_THR_MIN=0.00`
- `FW_T_SPDWEIGHT=1.5`
- `FW_T_RLL2THR=2.0`
- `FW_PSP_OFF=4.0`
- `FW_R_LIM=28`
- `VT_F_TRANS_DUR=8.0`
- `VT_F_TRANS_THR=0.55`
- `VT_TRANS_MIN_TM=6.0`
- `VT_F_TR_OL_TM=12.0`
- `VT_TRANS_TIMEOUT=30`
- `VT_B_TRANS_DUR=8.0`
- `VT_B_DEC_MSS=1.2`
- `VT_B_TRANS_RAMP=6.0`
- `VT_FW_MIN_ALT=30`
- `VT_QC_T_ALT_LOSS=35`
- `NPFG_PERIOD=34.0`
- `NPFG_DAMPING=1.0`
- `NPFG_ROLL_TC=1.5`
- `NAV_LOITER_RAD=900`
- `RTL_LOITER_RAD=900`
- `RTL_RETURN_ALT=100`
- `RTL_DESCEND_ALT=75`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.44`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`
- `Airspeed source=body_axis pitotAxisBody=-Z`

In TruthCapture, confirm:

- `sim/flightmodel/weight/m_total` is about `5.4 kg` loaded
- `sim/aircraft/weight/acf_m_empty` is about `5 kg`
- Optional design check:
  `python3 tools/analyze_qtailsitter_design.py aircraft/QuadTailsitter/QuadTailsitter.acf --truth <truth-folder-or-zip>`

## Pause / Dialog Check

For this test, do one short non-flight check before the flight sequence:

1. Connect PX4 and wait for Ready for takeoff.
2. Pause X-Plane or open a small X-Plane aircraft/weight dialog for `5-10 s`.
3. Close/unpause and verify PX4 continues receiving telemetry.
4. In X-Plane `Log.txt`, expect `[BRIDGE_PAUSE]` and `[BRIDGE_RESUME]` lines.
5. If PX4 reports persistent baro stale, high accelerometer bias, EKF missing
   data, or vertical velocity instability after unpause, stop and send the logs
   before flying.

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The next log should be used to verify:

- The ULog initial parameters match this card, especially the canted
  `CA_ROTOR*_AX/AY/AZ` values. If they are all `AX=0`, `AY=0`, `AZ=-1`, stop:
  the run is using stale PX4 parameters.
- Go-To does not repeat qtail20's `14 m/s` overspeed. Target acceptance for this
  slice is actual horizontal speed below about `6 m/s`, pitch not running away
  beyond about `35 deg`, and no sustained pitch/roll error or motor saturation.
- Normal deceleration at the final Go-To target is not treated as a failure.
- Orbit entry does not recreate the qtail8 yaw spin or vertical sink. Skip
  orbit if the two Go-To commands are not clean.
- The first transition should remain in transition until the body-axis
  calibrated airspeed reaches roughly `VT_ARSP_TRANS=24 m/s`.
- In the ULog, `airspeed_validated.calibrated_airspeed_m_s` should be positive
  in FW and should broadly match truth true airspeed. QGC should no longer show
  the large negative FW airspeed seen in qtail11.
- FW path following is judged against the new `28 m/s` trim target and `900 m`
  loiter radius. If speed still runs into the `40+ m/s` range, record it but do
  not tighten the radius in the same run.
