# QuadTailsitter X-Plane 12 Hover, Go-To, and Orbit Workflow

This card is for the next controlled QuadTailsitter validation after `qtail9`.
qtail9 was the best multicopter-mode flight so far. The reported Go-To
brake/continue behavior was mostly expected PX4 point-reposition behavior:
Go-To decelerates near the target instead of blending separate points as a
continuous path. A smaller tuning issue remains because pitch lagged the
trajectory setpoint in parts of the Go-To. The next test should verify v3.4.31
in Position, Go-To, and Orbit before transition work resumes.

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
3. Command one modest Go-To movement around `3 m/s`.
4. Command one faster Go-To around `4 m/s`.
5. If attitude tracking, altitude, and motor headroom remain clean, command one
   `5 m/s` Go-To.
6. At `20 m` AGL or higher, command one `40-50 m` Orbit.
   - If QGC exposes Orbit yaw behavior, use hold initial heading or tangent
     heading for this test. If it does not, use the default center-facing Orbit
     and keep the radius at least `40 m`.
7. RTL or Land and wait `10-15 s` after disarm before stopping PX4.

Do not command forward transition on this validation run. Abort immediately if
roll or pitch exceeds about `35 deg`, stays above about `25 deg`, yaw diverges
by more than about `45 deg` and keeps growing, an attitude warning appears,
motors sit at min/max for more than a brief recovery pulse, persistent
unallocated torque is visible, or rapid uncontrolled climb/descent starts.

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
- `MC_YAW_P=0.75`
- `MC_YAW_WEIGHT=0.35`
- `MC_YAWRATE_P=0.16`
- `MC_YAWRATE_I=0.015`
- `MC_YAWRATE_D=0.015`
- `MC_YAWRATE_K=1.15`
- `MC_YAWRATE_MAX=60`
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
- `MPC_XY_P=0.14`
- `MPC_Z_P=1.00`
- `MPC_Z_VEL_P_ACC=2.0`
- `MPC_Z_VEL_I_ACC=0.5`
- `MPC_Z_VEL_D_ACC=0.5`
- `MPC_XY_CRUISE=5.0`
- `MPC_XY_VEL_MAX=5.0`
- `MPC_VEL_MANUAL=4.0`
- `MPC_XY_ERR_MAX=10`
- `MPC_ACC_HOR=2.2`
- `MPC_ACC_HOR_MAX=2.5`
- `MPC_JERK_AUTO=1.5`
- `MPC_JERK_MAX=2.5`
- `MIS_TAKEOFF_ALT=1.5`
- `MPC_Z_V_AUTO_UP=1.0`
- `MPC_Z_VEL_MAX_UP=1.5`
- `MPC_ACC_UP_MAX=2.2`
- `MPC_TKO_SPEED=1.0`
- `MPC_TKO_RAMP_T=1.5`
- `MPC_TILTMAX_AIR=32.0`
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

- `px4xplane: Version: v3.4.31`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The next log should be used to verify that Go-To can reach `4 m/s`, and then
`5 m/s` if the first leg is clean, without sustained pitch/roll error, motor
saturation, or altitude loss. Do not treat normal deceleration at the final
Go-To point as a failure. Orbit entry must not create the qtail8 yaw spin or
vertical sink. Transition, canted-motor geometry, and any physical rescale work
come only after this multicopter hover/Go-To/Orbit loop is stable.
