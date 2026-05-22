# QuadTailsitter X-Plane 12 Hover, Go-To, and Orbit Workflow

This card is for the next controlled QuadTailsitter validation after `qtail8`.
qtail8 showed that the vehicle can handle more multicopter-mode speed than the
first hover-recovery values, but the live test rolled over after an Orbit
command with high tilt, high speed, and aggressive yaw tuning. The next test
should verify v3.4.30 in Position, Go-To, and Orbit before transition work
resumes.

## Setup

1. Install the px4xplane package and set `px4xplane/64/config.ini` to:
   - `config_name = QuadTailsitter`
2. Install the packaged `QuadTailsitter` aircraft folder into X-Plane.
3. Install the PX4 airframe file:
   - `5021_xplane_qtailsitter`
4. Run PX4 with:
   - `make px4_sitl_default xplane_qtailsitter`
   - `SYS_AUTOSTART` must be `5021`
5. Run `make distclean` once after replacing or pulling the airframe file.
6. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off and let PX4 reach QGC's commanded takeoff altitude.
2. Hold in multicopter mode for `15-20 s`.
3. Command one modest Go-To movement.
4. Command one faster Go-To, about `2.5-3.0 m/s`.
5. At `20 m` AGL or higher, command one `40-50 m` Orbit.
   - If QGC exposes Orbit yaw behavior, use hold initial heading or tangent
     heading for this test. If it does not, use the default center-facing Orbit
     and keep the radius at least `40 m`.
6. RTL or Land and wait `10-15 s` after disarm before stopping PX4.

Do not command forward transition on this validation run. Abort immediately if
roll or pitch exceeds about `35 deg`, stays above about `25 deg`, yaw diverges
by more than about `45 deg` and keeps growing, an attitude warning appears,
motors sit at min/max for more than a brief recovery pulse, persistent
unallocated torque is visible, or rapid uncontrolled climb/descent starts.

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
- `MC_ROLL_P=0.8`
- `MC_PITCH_P=0.8`
- `MC_YAW_P=0.55`
- `MC_YAW_WEIGHT=0.35`
- `MC_YAWRATE_P=0.10`
- `MC_YAWRATE_I=0.015`
- `MC_YAWRATE_D=0.005`
- `MC_YAWRATE_K=1.10`
- `MC_YAWRATE_MAX=60`
- `MC_ROLLRATE_P=0.09`
- `MC_PITCHRATE_P=0.09`
- `MC_ROLLRATE_K=0.45`
- `MC_PITCHRATE_K=0.60`
- `MC_ROLLRATE_MAX=70`
- `MC_PITCHRATE_MAX=70`
- `MPC_THR_HOVER=0.27`
- `MPC_USE_HTE=0`
- `MPC_XY_P=0.10`
- `MPC_Z_P=0.80`
- `MPC_Z_VEL_P_ACC=2.5`
- `MPC_Z_VEL_I_ACC=0.6`
- `MPC_Z_VEL_D_ACC=0.4`
- `MPC_XY_CRUISE=3.0`
- `MPC_XY_VEL_MAX=3.5`
- `MPC_VEL_MANUAL=3.0`
- `MPC_ACC_HOR=2.0`
- `MPC_ACC_HOR_MAX=2.0`
- `MPC_JERK_AUTO=1.2`
- `MPC_JERK_MAX=2.0`
- `MIS_TAKEOFF_ALT=1.5`
- `MPC_Z_V_AUTO_UP=1.0`
- `MPC_Z_VEL_MAX_UP=1.5`
- `MPC_ACC_UP_MAX=2.2`
- `MPC_TKO_SPEED=1.0`
- `MPC_TKO_RAMP_T=1.5`
- `MPC_TILTMAX_AIR=25.0`
- `MPC_YAW_MODE=0`
- `MPC_YAWRAUTO_MAX=35`
- `MPC_YAWRAUTO_ACC=12`
- `MIS_YAW_ERR=30`
- `MIS_YAW_TMT=5`
- `MC_ORBIT_YAW_MOD=1`
- `LNDMC_Z_VEL_MAX=0.25`
- `FW_USE_AIRSPD=1`
- `ASPD_DO_CHECKS=1`
- `SYS_HAS_NUM_ASPD=0`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.30`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The next log should be used to verify that Go-To can reach about `2.5-3.0 m/s`
without sustained pitch/roll error, Orbit entry does not create the qtail8 yaw
spin or vertical sink, yaw remains allocated, motors are not constantly
saturated, and land detection remains safe. Transition, canted-motor geometry,
and any physical rescale work come only after this multicopter
hover/Go-To/Orbit loop is stable.
