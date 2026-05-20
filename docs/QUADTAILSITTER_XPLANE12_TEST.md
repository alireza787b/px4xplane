# PB50 QuadTailsitter X-Plane 12 First-Test Workflow

This card is for the first controlled PB50 QuadTailsitter validation after the
Alia and Ehang milestone. The PB50 model is a small four-motor tailsitter with
no control surfaces, so the first goal is a clean hover, transition, fixed-wing
hold/orbit, back-transition, and landing log, not final tuning.

## Setup

1. Install the px4xplane package and set `px4xplane/64/config.ini` to:
   - `config_name = QuadTailsitter`
2. Install the packaged `PB50_QuadTailsitter` aircraft folder into X-Plane.
3. Install the PX4 airframe file:
   - `5021_xplane_qtailsitter`
4. Run PX4 with:
   - `make px4_sitl_default xplane_qtailsitter`
   - `SYS_AUTOSTART` must be `5021`
5. Run `make distclean` once after replacing or pulling the airframe file.
6. Start XPlaneTruthCapture before connecting PX4.

## Scenario

Use calm weather and model calculations per frame `6`.

1. Take off to `50-60 m`.
2. Hold in multicopter mode for at least `20 s`.
3. Apply small manual roll, pitch, and yaw checks if the hover is stable.
4. Command forward transition.
5. In fixed-wing mode, hold straight flight briefly, then use a wide loiter or
   RTL with `120 m` or larger radius.
6. Command RTL/back-transition and landing.
7. Wait `10-15 s` after disarm before stopping PX4.

Abort the run if the vehicle shows immediate large hover oscillation, wrong-axis
response, no airspeed, or continuous altitude loss after fixed-wing switch.

## Parameter Sanity Check

Before judging the run, confirm these defaults in the ULog:

- `SYS_AUTOSTART=5021`
- `MAV_TYPE=20`
- `CA_AIRFRAME=4`
- `CA_SV_CS_COUNT=0`
- `MC_AIRMODE=2`
- `MC_ROLL_P=3.0`
- `MC_PITCH_P=3.0`
- `MC_ROLLRATE_P=0.30`
- `MC_PITCHRATE_P=0.30`
- `FW_AIRSPD_MIN=14.0`
- `FW_AIRSPD_TRIM=18.0`
- `FW_AIRSPD_MAX=24.0`
- `FW_USE_AIRSPD=1`
- `ASPD_DO_CHECKS=1`
- `SYS_HAS_NUM_ASPD=0`
- `VT_ARSP_TRANS=15.0`
- `VT_F_TRANS_DUR=1.5`
- `VT_B_TRANS_DUR=5.0`
- `VT_FW_DIFTHR_EN=7`
- `NAV_LOITER_RAD=120.0`
- `RTL_LOITER_RAD=120.0`
- `EKF2_BARO_NOISE=1.0`
- `CAL_BARO1_PRIO=0`
- `IMU_GYRO_RATEMAX=200`
- `IMU_INTEG_RATE=200`

In X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v3.4.22`
- `Config Name: PB50 QuadTailsitter`
- the connection HUD shows `Airframe: PB50 QuadTailsitter`

## Log Package

Save and send:

- PX4 `.ulg`
- PX4 terminal output
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip

The first PB50 log should be used to verify motor order, body-axis signs,
airspeed validity, hover control, differential-thrust fixed-wing authority, and
transition timing before any aggressive performance tuning.
