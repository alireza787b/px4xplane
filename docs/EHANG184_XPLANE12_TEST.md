# Ehang 184 X-Plane 12 Validation Workflow

This card is for the first controlled final-validation run of the Ehang 184
X-Plane multicopter after the Alia tuning lessons were applied.

## Scenario

1. Install px4xplane and set `px4xplane/64/config.ini` to:
   - `config_name = ehang184`
2. Install XPlaneTruthCapture `v0.1.7` or newer and start recording before PX4
   connects.
3. Run PX4 with the validation launcher and select `xplane_ehang184`:
   - `px4xplane --validation --reset-config`
   - `SYS_AUTOSTART` must be `5010`
4. Confirm the printed validation stack shows official PX4 plus the pending
   EKF-GSF, Standard VTOL, TECS, and tailsitter PR guards unless you are
   deliberately testing `--official`.
5. Fly:
   - takeoff to about `30 m`
   - hover for at least `20 s`
   - yaw left/right with no aggressive heading snap
   - translate forward/back/left/right
   - climb and descend
   - Orbit with a commanded radius of `80-100 m`
   - mission waypoint or auto reposition if QGC switches the vehicle into an
     auto mode that consumes the reposition setpoint
   - RTL and landing
6. Wait `10-15 s` after disarm before stopping PX4.
7. Save PX4 ULog, PX4 terminal output, X-Plane `Log.txt`, and the
   XPlaneTruthCapture folder/zip.

## Parameter Sanity Check

Confirm these values in the ULog before judging the result:

- `SYS_AUTOSTART=5010`
- `MC_ROLL_P=1.0`
- `MC_PITCH_P=1.0`
- `MC_ROLLRATE_K=3.0`
- `MC_PITCHRATE_K=2.05`
- `MC_YAW_P=0.6`
- `MPC_XY_P=0.05`
- `MPC_XY_VEL_P_ACC=1.2`
- `MPC_XY_VEL_D_ACC=0.8`
- `MPC_XY_ERR_MAX=10.0`
- `MPC_XY_CRUISE=5.0`
- `MPC_XY_VEL_MAX=6.5`
- `MPC_VEL_MANUAL=5.0`
- `MPC_Z_V_AUTO_UP=2.0`
- `MPC_Z_V_AUTO_DN=1.2`
- `MPC_TKO_SPEED=2.0`
- `MPC_LAND_SPEED=0.6`
- `MPC_LAND_ALT1=8.0`
- `MPC_LAND_ALT2=3.0`
- `MPC_LAND_CRWL=0.30`
- `LNDMC_Z_VEL_MAX=0.24`
- `COM_DISARM_LAND=1.5`
- `MPC_ACC_HOR=1.5`
- `MPC_ACC_HOR_MAX=1.8`
- `MPC_ACC_UP_MAX=2.0`
- `MPC_ACC_DOWN_MAX=2.0`
- `MPC_JERK_AUTO=1.0`
- `MPC_JERK_MAX=2.0`
- `MPC_MAN_TILT_MAX=25.0`
- `MPC_TILTMAX_AIR=30.0`
- `MPC_MAN_Y_MAX=15.0`
- `MPC_YAWRAUTO_MAX=20.0`
- `NAV_ACC_RAD=12.0`
- `NAV_MC_ALT_RAD=2.0`
- `CAL_ACC0_PRIO=50`
- `EKF2_GPS_P_NOISE=1.5`
- PX4 default baro noise/gate values are used with GPS height reference
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`
- `SENS_IMU_MODE=0`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v4.1.1`
- `config_name = ehang184` in `px4xplane/64/config.ini`
- the X-Plane connection HUD shows `Airframe: Ehang 184` while waiting for PX4

The `evtol1` Ehang log showed QGC `MAV_CMD_DO_REPOSITION` commands accepted
while the vehicle stayed in Position mode and did not move horizontally. For the
next validation, use Orbit, RTL, or a mission/auto mode for objective path
tracking. A QGC "Go to location" test is still useful, but only record it as a
command/mode-behavior check unless PX4 changes into a mode that actually follows
the reposition setpoint.

The `evtol2`, `evtol3`, and `evtol4` orbit logs showed the vehicle initially starting well
outside the commanded circle. PX4 Orbit mode first captures the closest point on
the commanded circle, so a short S-turn/zigzag before the stable orbit is
expected when the command is sent from outside the radius. Judge the tune after
capture: radius hunting should be damped, pitch should not hit repeated
`+-30 deg` setpoints, and actual roll should stay well below the attitude
failure threshold.

For a large commanded orbit, visible roll can be very small. For example,
`5 m/s` around a `146 m` circle only requires about `1 deg` lateral tilt. That
is not a missing roll-channel symptom by itself.

## Acceptance

- No in-flight EKF, baro, GPS, or accelerometer-bias warnings.
- Hover position and altitude do not oscillate visibly.
- Yaw response is smooth and does not overshoot aggressively.
- Auto descent and landing are controlled, with no bounce or tip tendency.
- RTL returns and lands without failsafe.
- TruthCapture reports zero dropped rows and no sim-time reset.
