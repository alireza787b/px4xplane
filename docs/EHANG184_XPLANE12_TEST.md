# Ehang 184 X-Plane 12 Validation Workflow

This card is for the first controlled final-validation run of the Ehang 184
X-Plane multicopter after the Alia tuning lessons were applied.

## Scenario

1. Install px4xplane and set `px4xplane/64/config.ini` to:
   - `config_name = ehang184`
2. Install XPlaneTruthCapture `v0.1.7` or newer and start recording before PX4
   connects.
3. Run PX4 SITL with the Ehang airframe:
   - `make px4_sitl_default xplane_ehang184`
   - `SYS_AUTOSTART` must be `5010`
4. After changing the airframe file, run `make distclean` once or reset SITL
   parameters so saved state cannot mask the defaults.
5. Fly:
   - takeoff to about `30 m`
   - hover for at least `20 s`
   - yaw left/right with no aggressive heading snap
   - translate forward/back/left/right
   - climb and descend
   - auto loiter/hold or mission waypoint if available
   - RTL and landing
6. Wait `10-15 s` after disarm before stopping PX4.
7. Save PX4 ULog, PX4 terminal output, X-Plane `Log.txt`, and the
   XPlaneTruthCapture folder/zip.

## Parameter Sanity Check

Confirm these values in the ULog before judging the result:

- `SYS_AUTOSTART=5010`
- `MPC_XY_CRUISE=8.0`
- `MPC_XY_VEL_MAX=12.0`
- `MPC_Z_V_AUTO_UP=2.0`
- `MPC_Z_V_AUTO_DN=1.0`
- `MPC_TKO_SPEED=1.2`
- `MPC_LAND_SPEED=0.6`
- `MPC_LAND_ALT1=8.0`
- `MPC_LAND_ALT2=3.0`
- `MPC_LAND_CRWL=0.25`
- `MPC_ACC_HOR=2.0`
- `MPC_ACC_HOR_MAX=2.0`
- `MPC_ACC_UP_MAX=2.0`
- `MPC_ACC_DOWN_MAX=2.0`
- `MPC_JERK_AUTO=1.0`
- `MPC_JERK_MAX=2.0`
- `MPC_MAN_TILT_MAX=25.0`
- `MPC_TILTMAX_AIR=30.0`
- `MPC_MAN_Y_MAX=15.0`
- `MPC_YAWRAUTO_MAX=20.0`
- `NAV_ACC_RAD=25.0`
- `NAV_MC_ALT_RAD=2.0`
- `EKF2_BARO_NOISE=1.0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.14`
- `config_name = ehang184` in `px4xplane/64/config.ini`

## Acceptance

- No in-flight EKF, baro, GPS, or accelerometer-bias warnings.
- Hover position and altitude do not oscillate visibly.
- Yaw response is smooth and does not overshoot aggressively.
- Auto descent and landing are controlled, with no bounce or tip tendency.
- RTL returns and lands without failsafe.
- TruthCapture reports zero dropped rows and no sim-time reset.
