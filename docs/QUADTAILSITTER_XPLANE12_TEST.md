# QuadTailsitter X-Plane 12 Final Polish Workflow

This card is for the next controlled QuadTailsitter closure validation after
`qtail27.zip`. v4.0.0 keeps the accepted `5 kg` 6S 3115-class aircraft target,
the stationary sensor/contact fixes, and the body-axis `-Z` virtual pitot.
qtail27 showed that manual RTL can trigger auto back-transition too low when
`RTL_DESCEND_ALT=75 m`, so this package raises the RTL altitude margin and adds
config-driven plugin camera views plus the upgraded editor/menu workflow that
does not depend on numpad quick-look bindings.

The next gate is a high-margin manual RTL plus mission validation. Do not use a
low-altitude VTOL Land approach yet; keep the approach/back-transition altitude
high enough that a failed recovery has margin.

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
     `python3 tools/check_px4_airframe_params.py <ulog> config/px4_params/5021_xplane_qtailsitter --param SYS_HAS_NUM_ASPD --param MPC_THR_HOVER --param CA_ROTOR0_AX --param CA_ROTOR1_AX --param MPC_XY_CRUISE --param MPC_Z_V_AUTO_UP --param VT_ARSP_TRANS --param FW_AIRSPD_TRIM --param FW_R_LIM --param VT_FW_DIFTHR_S_R --param VT_FW_DIFTHR_S_Y --param NPFG_PERIOD --param NAV_LOITER_RAD --param VT_B_TRANS_DUR --param FD_FAIL_P`
7. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off and let PX4 reach QGC's commanded takeoff altitude.
2. Hold in multicopter mode for `15-20 s`.
3. Command one modest Go-To movement. The intended MC cruise is `3.5 m/s`.
4. Command one longer Go-To and watch for actual speed overshoot.
5. Climb to at least `180 m` AGL before transition.
6. Command one straight forward transition. Let it stabilize in fixed-wing mode.
7. Fly one straight leg or a large-radius mission/loiter. The default FW
   loiter/RTL radius is now `1300 m`; do not use tight manual orbit radii for
   this gate.
8. Use a mission VTOL Land item only with a high approach/back-transition
   altitude. If you want to reduce the altitude later, do it only after one
   clean qtail26-style run.
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

Keep fixed-wing waypoint spacing at least `1300 m`, avoid tight waypoint
clusters, and do not use a `50 m` VTOL Land approach altitude for public demos.
qtail27 proved that `75 m` RTL descent altitude is not enough margin for this
tailsitter if auto RTL starts back-transition from fixed-wing energy. v4.0.0
uses `RTL_RETURN_ALT=180` and `RTL_DESCEND_ALT=150` so manual RTL starts the
recovery higher while preserving the qtail26/qtail27 accepted flight-control
baseline.

The current QuadTailsitter contact gear is intentionally fixed for physics
stability. v4.0.0 hides the large rendered gear geometry while keeping the
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
- `MC_PITCH_P=0.58`
- `MC_YAW_P=0.65`
- `MC_YAW_WEIGHT=0.35`
- `MC_YAWRATE_P=0.20`
- `MC_YAWRATE_I=0.015`
- `MC_YAWRATE_D=0.025`
- `MC_YAWRATE_K=1.10`
- `MC_YAWRATE_MAX=55`
- `MC_ROLLRATE_P=0.10`
- `MC_PITCHRATE_P=0.115`
- `MC_ROLLRATE_D=0.0008`
- `MC_PITCHRATE_D=0.0012`
- `MC_ROLLRATE_K=1.00`
- `MC_PITCHRATE_K=0.92`
- `MC_ROLLRATE_MAX=80`
- `MC_PITCHRATE_MAX=110`
- `MPC_THR_HOVER=0.22`
- `MPC_THR_MIN=0.08`
- `MPC_THR_XY_MARG=0.4`
- `MPC_XY_P=0.18`
- `MPC_Z_P=1.00`
- `MPC_Z_VEL_P_ACC=2.0`
- `MPC_Z_VEL_I_ACC=0.5`
- `MPC_Z_VEL_D_ACC=0.6`
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
- `MPC_YAWRAUTO_MAX=25`
- `MPC_YAWRAUTO_ACC=8`
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
- `VT_ARSP_BLEND=17`
- `VT_ARSP_TRANS=22`
- `FW_AIRSPD_MIN=19`
- `FW_AIRSPD_TRIM=27`
- `FW_AIRSPD_MAX=36`
- `FW_ARSP_SCALE_EN=0`
- `FW_THR_TRIM=0.24`
- `FW_THR_MAX=0.55`
- `FW_THR_MIN=0.16`
- `FW_THR_SLEW_MAX=0.30`
- `FW_T_ALT_TC=6.0`
- `FW_T_SPDWEIGHT=0.55`
- `FW_T_RLL2THR=2.0`
- `FW_T_CLMB_R_SP=2.0`
- `FW_T_SINK_R_SP=2.0`
- `FW_T_VERT_ACC=1.8`
- `FW_PSP_OFF=2.0`
- `FW_P_TC=0.75`
- `FW_P_LIM_MAX=28`
- `FW_P_RMAX_POS=45`
- `FW_P_RMAX_NEG=35`
- `FW_PR_P=0.30`
- `FW_PR_I=0.06`
- `FW_PR_FF=0.12`
- `FW_R_TC=1.5`
- `FW_R_LIM=16`
- `FW_R_RMAX=35`
- `FW_PN_R_SLEW_MAX=10`
- `FW_RR_P=0.07`
- `FW_RR_I=0.02`
- `FW_RR_FF=0.03`
- `FW_YR_P=0.01`
- `VT_F_TRANS_DUR=10.0`
- `VT_F_TRANS_THR=0.36`
- `VT_TRANS_MIN_TM=8.0`
- `VT_F_TR_OL_TM=14.0`
- `VT_TRANS_TIMEOUT=30`
- `VT_B_TRANS_DUR=5.0`
- `VT_B_DEC_MSS=1.0`
- `VT_B_DEC_I=0.12`
- `VT_B_TRANS_RAMP=1.5`
- `VT_FW_DIFTHR_S_R=0.35`
- `VT_FW_DIFTHR_S_P=0.48`
- `VT_FW_DIFTHR_S_Y=0.25`
- `VT_FW_MIN_ALT=30`
- `VT_QC_T_ALT_LOSS=35`
- `NPFG_PERIOD=55.0`
- `NPFG_DAMPING=1.0`
- `NPFG_ROLL_TC=2.4`
- `NPFG_SW_DST_MLT=0.70`
- `NAV_ACC_RAD=120`
- `NAV_FW_ALT_RAD=40`
- `NAV_LOITER_RAD=1300`
- `RTL_LOITER_RAD=1300`
- `RTL_RETURN_ALT=180`
- `RTL_DESCEND_ALT=150`
- `FD_FAIL_R=85`
- `FD_FAIL_P=85`
- `FD_FAIL_R_TTRI=1.0`
- `FD_FAIL_P_TTRI=1.0`
- `EKF2_GPS_P_NOISE=1.5`
- PX4 default baro noise/gate values are used with GPS height reference
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`
- `SENS_IMU_MODE=0`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v4.0.2`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`
- `Airspeed source=body_axis pitotAxisBody=-Z`
- `Loaded 3 camera view(s) for QuadTailsitter`

In the installed aircraft folder, confirm these presentation assets are present:

- `QuadTailsitter_icon11.png`
- `QuadTailsitter_icon11_thumb.png`
- `README.md`

The plugin now provides a generic `PX4 X-Plane > Camera Views` menu. Camera
presets are read from the active airframe's optional `cameraViews` config value:

`Label|forward_m|right_m|up_m|pitch_offset_deg|heading_offset_deg|roll_offset_deg|zoom`

Every packaged airframe now ships forward, down, and chase camera presets.
QuadTailsitter uses tailsitter-specific offsets:

- `Nose / FPV`: in MC hover it points upward; in fixed-wing it points forward
  along the virtual pitot axis.
- `Belly / Down`: in fixed-wing it looks down toward the ground.
- `Chase`: rear engineering chase view.

These bindable X-Plane commands are generic and reusable for any configured
airframe:

- `px4xplane/camera/view_1`
- `px4xplane/camera/view_2`
- `px4xplane/camera/view_3`
- `px4xplane/camera/release`: return camera control to X-Plane.

Bind these commands to your preferred keys in X-Plane's keyboard settings if
your keyboard does not have a numpad. The aircraft package also keeps
quick-look presets in `QuadTailsitter_prefs.txt` as a fallback:

- quick look `#1`: nose/FPV camera
- quick look `#2`: belly/down-looking camera
- quick look `#3`: rear engineering chase view

In TruthCapture, confirm:

- `sim/flightmodel/weight/m_total` is about `5.4 kg` loaded
- `sim/aircraft/weight/acf_m_empty` is about `5 kg`
- Optional design check:
  `python3 tools/analyze_qtailsitter_design.py aircraft/QuadTailsitter/QuadTailsitter.acf --truth <truth-folder-or-zip>`

## qtail26 Baseline

The qtail26 mission run loaded the intended PX4 values and is the
baseline for judging this package:

- TruthCapture: `45,723` rows, `0` dropped rows, `0` sim-time resets, about
  `88 Hz` mean callback rate.
- Fixed-wing phase: about `28.7 m/s` TAS median, altitude held near `100 m`,
  and motor mean p50 about `0.10`.
- Back-transition and landing were safe, with no failure-detector flags.
- Remaining issue: back-transition exited with high horizontal speed and
  ballooned to about `202 m AGL` before MC descent.
- Estimator health was clean: no baro stale, vertical velocity instability,
  accelerometer-bias warning, or pause/FPS sensor regression.

v4.0.0 preserves the qtail26/qtail27 bridge, ACF, and flight-control baseline.
The only flight-param change is the RTL return/descent altitude margin for
manual RTL from fixed-wing mode.

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
  `CA_ROTOR*_AX/AY/AZ` values and the v4.0.0 RTL/FW/back-transition values. If the
  rotor axes are all `AX=0`, `AY=0`, `AZ=-1`, or if `FW_R_LIM` is still `32`,
  stop: the run is using stale PX4 parameters.
- Go-To does not repeat qtail20's `14 m/s` overspeed. Target acceptance for this
  slice is actual horizontal speed below about `6 m/s`, pitch not running away
  beyond about `35 deg`, and no sustained pitch/roll error or motor saturation.
- Normal deceleration at the final Go-To target is not treated as a failure.
- Manual RTL should not descend below the raised `RTL_DESCEND_ALT=150` before
  auto back-transition. If it starts back-transition near `75 m AGL`, stop and
  check for stale PX4 parameters.
- The first transition should remain in transition until the body-axis
  calibrated airspeed reaches roughly `VT_ARSP_TRANS=22 m/s`.
- In the ULog, `airspeed_validated.calibrated_airspeed_m_s` should be positive
  in FW and should broadly match truth true airspeed. QGC should no longer show
  the large negative FW airspeed seen in qtail11.
- FW energy behavior is judged against the `27 m/s` trim target. If speed still
  runs into the `40+ m/s` range or altitude/throttle oscillates, record it and
  send the logs before continuing to Orbit/RTL/VTOL Land tuning.
