# QuadTailsitter X-Plane 12 Hover-Recovery Workflow

This card is for the next controlled QuadTailsitter validation after `qtail2`.
The previous run still diverged in multicopter hover before any transition
tuning was valid, so the next test is intentionally hover-first.

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

1. Take off to only `2 m`.
2. Hold in multicopter mode for at least `20 s`.
3. If hover is stable, apply very small roll and pitch stick inputs or short
   Hold reposition commands. Keep angles below about `10 deg`.
4. Land and wait `10-15 s` after disarm before stopping PX4.

Do not command forward transition on this first recovery run. Abort immediately
if roll or pitch exceeds about `15 deg`, diagonal motor saturation appears,
attitude warnings appear, or rapid uncontrolled climb/descent starts.

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
- `MC_AIRMODE=0`
- `MC_ROLL_P=0.8`
- `MC_PITCH_P=0.8`
- `MC_ROLLRATE_P=0.08`
- `MC_PITCHRATE_P=0.08`
- `MC_ROLLRATE_K=0.30`
- `MC_PITCHRATE_K=0.30`
- `MC_ROLLRATE_MAX=70`
- `MC_PITCHRATE_MAX=70`
- `MPC_THR_HOVER=0.22`
- `MPC_USE_HTE=0`
- `MPC_TKO_SPEED=0.6`
- `MPC_TKO_RAMP_T=5.0`
- `MPC_TILTMAX_AIR=15.0`
- `LNDMC_Z_VEL_MAX=0.20`
- `FW_USE_AIRSPD=1`
- `ASPD_DO_CHECKS=1`
- `SYS_HAS_NUM_ASPD=0`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.24`
- `Config Name: QuadTailsitter`
- the connection HUD shows `Airframe: QuadTailsitter`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The next log should be used to verify hover motor order, body-axis signs,
disabled X-Plane internal engine stabilization, diagonal motor saturation
recovery, and safe land detection. Transition and fixed-wing tuning come only
after the multicopter hover loop is stable.
