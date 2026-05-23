# QuadTailsitter X-Plane 12 Hover, Go-To, Orbit, and First Transition Workflow

This card is for the next controlled QuadTailsitter validation after the
`qtail11.zip` run. qtail11 confirmed that the body-axis pitot projection on
`-Z` is positive and close to truth airspeed in fixed-wing mode. The next run
therefore uses PX4 airspeed feedback for transition and TECS instead of the
previous airspeedless/open-loop transition policy.

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
5. Force a clean PX4 parameter load before the test. Use one of:
   - preferred: `make distclean`, then rebuild/run `xplane_qtailsitter`
   - targeted: delete `build/px4_sitl_default/rootfs/parameters.bson` and
     `build/px4_sitl_default/rootfs/parameters_backup.bson`
   - PX4 shell: `param reset_all`, then stop and restart PX4
6. Do not judge the test if the ULog shows `SYS_AUTOCONFIG=0` with old/manual
   values that do not match the sanity-check list below.
7. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off and let PX4 reach QGC's commanded takeoff altitude.
2. Hold in multicopter mode for `15-20 s`.
3. Command one modest Go-To movement around `3 m/s`.
4. Command one faster Go-To around `4-5 m/s`.
5. At `20 m` AGL or higher, command one `40-50 m` multicopter Orbit.
   - If QGC exposes Orbit yaw behavior, use hold initial heading or tangent
     heading for this test. If it does not, use the default center-facing Orbit
     and keep the radius at least `40 m`.
6. If the MC phase is clean, climb to at least `80 m` AGL and command one
   forward transition while pointed into open space.
7. If it reaches fixed-wing state cleanly, keep it straight and shallow for
   `10-15 s`; if you test FW loiter, use at least `300 m` radius.
8. Back-transition manually before RTL if FW path following looks poor.
9. RTL or Land and wait `10-15 s` after disarm before stopping PX4.

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
- `CA_ROTOR0_CT=2.0`, `CA_ROTOR1_CT=2.0`
- `CA_ROTOR2_CT=2.0`, `CA_ROTOR3_CT=2.0`
- `CA_ROTOR0_KM=0.04`, `CA_ROTOR1_KM=0.04`
- `CA_ROTOR2_KM=-0.04`, `CA_ROTOR3_KM=-0.04`
- `MC_AIRMODE=2`
- `MC_ROLL_P=0.9`
- `MC_PITCH_P=0.9`
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
- `MC_PITCHRATE_D=0.0008`
- `MC_ROLLRATE_K=1.00`
- `MC_PITCHRATE_K=1.00`
- `MC_ROLLRATE_MAX=80`
- `MC_PITCHRATE_MAX=80`
- `MPC_THR_HOVER=0.27`
- `MPC_USE_HTE=0`
- `MPC_XY_P=0.18`
- `MPC_Z_P=1.00`
- `MPC_Z_VEL_P_ACC=2.0`
- `MPC_Z_VEL_I_ACC=0.5`
- `MPC_Z_VEL_D_ACC=0.5`
- `MPC_XY_CRUISE=5.0`
- `MPC_XY_VEL_MAX=5.0`
- `MPC_VEL_MANUAL=5.0`
- `MPC_XY_ERR_MAX=10`
- `MPC_ACC_HOR=1.8`
- `MPC_ACC_HOR_MAX=2.2`
- `MPC_JERK_AUTO=1.2`
- `MPC_JERK_MAX=2.0`
- `MIS_TAKEOFF_ALT=1.5`
- `MPC_Z_V_AUTO_UP=1.0`
- `MPC_Z_VEL_MAX_UP=1.5`
- `MPC_ACC_UP_MAX=2.2`
- `MPC_TKO_SPEED=1.0`
- `MPC_TKO_RAMP_T=1.5`
- `MPC_MAN_TILT_MAX=35.0`
- `MPC_TILTMAX_AIR=40.0`
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
- `LNDMC_Z_VEL_MAX=0.30`
- `FW_USE_AIRSPD=1`
- `ASPD_DO_CHECKS=0`
- `SYS_HAS_NUM_ASPD=1`
- `VT_ARSP_BLEND=14`
- `VT_ARSP_TRANS=20`
- `FW_AIRSPD_MIN=16`
- `FW_AIRSPD_TRIM=24`
- `FW_AIRSPD_MAX=35`
- `FW_THR_TRIM=0.12`
- `FW_THR_MAX=0.45`
- `FW_R_LIM=35`
- `VT_F_TRANS_DUR=8.0`
- `VT_F_TRANS_THR=0.45`
- `VT_TRANS_MIN_TM=6.0`
- `VT_F_TR_OL_TM=12.0`
- `VT_TRANS_TIMEOUT=30`
- `VT_B_TRANS_DUR=8.0`
- `VT_B_DEC_MSS=1.2`
- `VT_B_TRANS_RAMP=6.0`
- `VT_FW_MIN_ALT=40`
- `VT_QC_T_ALT_LOSS=35`
- `NAV_LOITER_RAD=300`
- `RTL_LOITER_RAD=300`
- `RTL_RETURN_ALT=80`
- `RTL_DESCEND_ALT=60`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.34`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`
- `Airspeed source=body_axis pitotAxisBody=-Z`

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The next log should be used to verify:

- Go-To reaches `4-5 m/s` without sustained pitch/roll error or motor
  saturation.
- Normal deceleration at the final Go-To target is not treated as a failure.
- Orbit entry does not recreate the qtail8 yaw spin or vertical sink.
- The first transition should remain in transition until the body-axis airspeed
  reaches roughly `VT_ARSP_TRANS=20 m/s`.
- In the ULog, `airspeed_validated.calibrated_airspeed_m_s` should be positive
  in FW and should broadly match truth true airspeed. QGC should no longer show
  the large negative FW airspeed seen in qtail11.
- FW path following is judged against the new airspeed target and `300 m`
  loiter radius. If speed still runs into the `40-50 m/s` range, record it but
  do not tighten the radius in the same run.
