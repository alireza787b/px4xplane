# QuadTailsitter X-Plane 12 FW Energy and Back-Transition Workflow

This card is for the next controlled QuadTailsitter validation after the
`qtail23.zip` mission run. v3.4.47 keeps the `5 kg` 6S 3115-class aircraft
target, the v3.4.43 stationary sensor/contact fixes, the body-axis `-Z` virtual
pitot, and the qtail21/qtail22 multicopter baseline. qtail23 proved that the
mission altitude frame was correct, but also showed a fixed-wing energy/lateral
control problem and an unsafe VTOL Land back-transition entry.

The next gate is fixed-wing stabilization and a high-margin back-transition. A
full VTOL Land item may be tested only after one clean manual back-transition at
high altitude in the same run.

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
   below. qtail20 proved that stale SITL `parameters.bson` can override the
   airframe `set-default` values even when X-Plane is running the right plugin.
   - After the run, use:
     `python3 tools/check_px4_airframe_params.py <ulog> config/px4_params/5021_xplane_qtailsitter --param SYS_HAS_NUM_ASPD --param MPC_THR_HOVER --param CA_ROTOR0_AX --param CA_ROTOR1_AX --param MPC_XY_CRUISE --param MPC_Z_V_AUTO_UP --param VT_ARSP_TRANS --param FW_AIRSPD_TRIM --param FW_T_SPDWEIGHT --param FW_R_LIM --param VT_FW_DIFTHR_S_R --param NPFG_PERIOD --param NAV_LOITER_RAD --param VT_B_TRANS_DUR`
7. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off and let PX4 reach QGC's commanded takeoff altitude.
2. Hold in multicopter mode for `15-20 s`.
3. Command one modest Go-To movement. The intended MC cruise is `3.5 m/s`.
4. Command one longer Go-To and watch for actual speed overshoot.
5. If MC actual horizontal speed remains below about `6 m/s` and attitude
   tracking is calm, climb to at least `150 m` AGL before transition.
6. Command one straight forward transition. Let it stabilize in fixed-wing mode.
7. Fly one straight leg or very large-radius loiter. The default FW loiter/RTL
   radius is now `900 m`; do not use tight manual orbit radii for this gate.
8. Manually back-transition while at least `150 m` AGL. If that is clean, a
   second run may use a VTOL Land item, but keep the approach/back-transition
   altitude high enough that a failed recovery has margin.
9. Land in MC and wait `10-15 s` after disarm before stopping PX4.

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

If you run an exploratory mission after a clean manual transition, keep
fixed-wing waypoint spacing at least `900 m`, avoid tight waypoint clusters, and
do not use a `50 m` VTOL Land approach altitude yet. qtail23 showed about `69 m`
of altitude loss before quad-chute with the previous back-transition timing.

The current QuadTailsitter contact gear is intentionally fixed for physics
stability. v3.4.46+ hides the large rendered gear geometry while keeping the
fixed contact pads active. True animated cosmetic retractable legs remain a
separate visual-asset slice.

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
- `MPC_Z_V_AUTO_UP=2.0`
- `MPC_Z_VEL_MAX_UP=2.4`
- `MPC_ACC_UP_MAX=2.2`
- `MPC_TKO_SPEED=1.2`
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
- `VT_ARSP_BLEND=20`
- `VT_ARSP_TRANS=24`
- `FW_AIRSPD_MIN=22`
- `FW_AIRSPD_TRIM=32`
- `FW_AIRSPD_MAX=45`
- `FW_THR_TRIM=0.20`
- `FW_THR_MAX=0.50`
- `FW_THR_MIN=0.00`
- `FW_T_SPDWEIGHT=0.8`
- `FW_T_RLL2THR=2.0`
- `FW_T_CLMB_R_SP=2.0`
- `FW_T_SINK_R_SP=2.0`
- `FW_PSP_OFF=2.0`
- `FW_R_TC=1.1`
- `FW_R_LIM=20`
- `FW_RR_P=0.14`
- `FW_RR_I=0.04`
- `FW_RR_FF=0.08`
- `FW_YR_P=0.05`
- `VT_F_TRANS_DUR=8.0`
- `VT_F_TRANS_THR=0.36`
- `VT_TRANS_MIN_TM=6.0`
- `VT_F_TR_OL_TM=12.0`
- `VT_TRANS_TIMEOUT=30`
- `VT_B_TRANS_DUR=6.0`
- `VT_B_DEC_MSS=1.6`
- `VT_B_DEC_I=0.10`
- `VT_B_TRANS_RAMP=3.0`
- `VT_FW_DIFTHR_S_R=0.70`
- `VT_FW_DIFTHR_S_P=0.85`
- `VT_FW_DIFTHR_S_Y=0.70`
- `VT_FW_MIN_ALT=30`
- `VT_QC_T_ALT_LOSS=35`
- `NPFG_PERIOD=40.0`
- `NPFG_DAMPING=1.0`
- `NPFG_ROLL_TC=1.6`
- `NPFG_SW_DST_MLT=0.70`
- `NAV_ACC_RAD=80`
- `NAV_FW_ALT_RAD=30`
- `NAV_LOITER_RAD=900`
- `RTL_LOITER_RAD=900`
- `RTL_RETURN_ALT=100`
- `RTL_DESCEND_ALT=75`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.47`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`
- `Airspeed source=body_axis pitotAxisBody=-Z`

In TruthCapture, confirm:

- `sim/flightmodel/weight/m_total` is about `5.4 kg` loaded
- `sim/aircraft/weight/acf_m_empty` is about `5 kg`
- Optional design check:
  `python3 tools/analyze_qtailsitter_design.py aircraft/QuadTailsitter/QuadTailsitter.acf --truth <truth-folder-or-zip>`

## qtail21 Baseline

The qtail21 acceptance run loaded the intended v3.4.44 PX4 values and is the
baseline for judging this package:

- TruthCapture: `22,696` rows, `0` dropped rows, `0` sim-time resets, about
  `80 Hz` mean callback rate.
- PX4 trajectory setpoint max horizontal speed: `3.5 m/s`.
- ULog actual horizontal speed max: about `4.1 m/s`.
- TruthCapture true airspeed max during MC: about `4.85 m/s`.
- Motor command peak: about `0.345`.
- Estimator timeout flags: `0`.

v3.4.47 preserves the qtail21/qtail22 MC lateral baseline and changes only MC
climb rate, fixed-wing energy/guidance, differential-thrust scaling, and
back-transition timing.

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
  `CA_ROTOR*_AX/AY/AZ` values and the v3.4.47 FW/back-transition values. If the
  rotor axes are all `AX=0`, `AY=0`, `AZ=-1`, or if `FW_R_LIM` is still `32`,
  stop: the run is using stale PX4 parameters.
- Go-To does not repeat qtail20's `14 m/s` overspeed. Target acceptance for this
  slice is actual horizontal speed below about `6 m/s`, pitch not running away
  beyond about `35 deg`, and no sustained pitch/roll error or motor saturation.
- Normal deceleration at the final Go-To target is not treated as a failure.
- Skip full VTOL Land missions until manual back-transition is clean.
- The first transition should remain in transition until the body-axis
  calibrated airspeed reaches roughly `VT_ARSP_TRANS=24 m/s`.
- In the ULog, `airspeed_validated.calibrated_airspeed_m_s` should be positive
  in FW and should broadly match truth true airspeed. QGC should no longer show
  the large negative FW airspeed seen in qtail11.
- FW energy behavior is judged against the `32 m/s` trim target. If speed still
  runs into the `40+ m/s` range or altitude/throttle oscillates, record it and
  send the logs before continuing to Orbit/RTL/VTOL Land tuning.
