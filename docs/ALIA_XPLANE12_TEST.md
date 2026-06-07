# Alia X-Plane 12 Validation Workflow

This workflow keeps the current bridge sensor contract as the baseline. Do not
change accel signs, GPS velocity units, GPS COG, or HIL_STATE units from a
single visual symptom. First compare truth capture, decoded bridge replay, and
PX4 ULog estimator topics.

## References

- PX4 Simulator MAVLink API: https://docs.px4.io/main/en/simulation/
- PX4 Generic Standard VTOL / QuadPlane tuning: https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration.html
- PX4 fixed-wing TECS/NPFG tuning: https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 multicopter PID tuning: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html

## Scenario

1. Install the px4xplane package with:
   - `px4xplane/64/win.xpl`
   - `px4xplane/64/config.ini`
   - `px4xplane/px4_airframes/5020_xplane_alia250`
   - this test note or an equivalent run card
   Set `px4xplane/64/config.ini` to `config_name = Alia250` before connecting.
2. Install XPlaneTruthCapture `v0.1.7` or newer. Confirm that starting capture
   changes the menu state to `Recording Active`; the optional overlay can be
   hidden with `overlay_enabled = false` in its `capture_config.ini`.
3. Use X-Plane 12 Alia with model calculations per frame set to `6`.
4. Start XPlaneTruthCapture before connecting PX4.
5. Run PX4 SITL with the Alia airframe. After changing the airframe file, use
   the setup script `distclean` option once, or otherwise reset SITL parameters,
   so saved `parameters.bson` state cannot mask the airframe defaults. This is
   required even if the file was updated by manually copying its contents into
   the PX4 checkout instead of using `git pull`.
6. Fly:
   - takeoff to about `100 m`
   - forward transition
   - RTL, circle, or loiter return
   - back-transition
   - landing and disarm
7. Wait `10-15 s` after disarm before stopping PX4. This makes shutdown-only
   vertical velocity warnings distinguishable from in-flight estimator issues.
8. Save:
   - XPlaneTruthCapture zip/folder
   - PX4 `.ulg`
   - PX4 CLI log
   - X-Plane `Log.txt`

## Replay

Decode truth-capture frames into bridge-contract rows:

```bash
python3 tools/replay_truth_capture.py /path/to/XPlaneTruthCapture-run.zip \
  --output /tmp/alia_bridge_replay.csv
```

This replay is a deterministic core-contract check. It intentionally omits
stochastic sensor noise so it can catch sign, unit, velocity, and scheduling
regressions before a full X-Plane/PX4 run.

For quick inspection:

```bash
python3 tools/replay_truth_capture.py /path/to/XPlaneTruthCapture-run.zip \
  --messages sensor,gps,state \
  --max-messages 20 \
  --format jsonl
```

## ULog Analysis

Install `pyulog` if needed:

```bash
python3 -m pip install pyulog
```

Analyze the estimator topics and warning windows:

```bash
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg --json > /tmp/alia_ulog_summary.json
```

The script requests these topics:

`estimator_status`, `estimator_status_flags`,
`estimator_innovation_test_ratios`, `estimator_aid_src_baro_hgt`,
`estimator_aid_src_gnss_vel`, `vehicle_local_position`,
`vehicle_air_data`, `sensor_baro`, `vehicle_gps_position`,
`commander_state`.

## Acceptance

- No in-flight EKF warnings.
- No GPS rejection bursts in estimator innovation/test-ratio topics.
- No barometer source switch during flight, or a clear ULog-backed explanation.
- Post-landing vertical velocity warning is eliminated or shown to be shutdown-only.
- Alia RTL/circle altitude and path deviation improve against the previous run.

## Parameter Sanity Check

Before a comparison run, confirm the PX4 log or shell reports `SYS_AUTOSTART=5020`.
The `alia-sitl2/19_07_20.ulg` retest artifact had `SYS_AUTOSTART=5010`, so it
was not valid evidence for Alia tuning.

For the current `v4.0.2` Alia milestone test,
verify these key defaults are
active in the PX4 ULog. If they still show the previous values, reset the SITL
parameter store and rerun before judging the package.

- `EKF2_MULTI_IMU=1`
- `SENS_IMU_MODE=0`
- `SENS_GPS0_DELAY=10`
- `SENS_GPS1_DELAY=10`
- `EKF2_GPS_P_NOISE=5.0`
- `EKF2_GPS_P_GATE=10.0`
- `EKF2_MAG_TYPE=1`
- `EKF2_ARSP_THR=50.0`
- `EKF2_GSF_TAS=0.0`
- `IMU_INTEG_RATE=200`
- `MPC_TKO_SPEED=3.0`
- `MPC_XY_ERR_MAX=10.0`
- `MPC_Z_VEL_MAX_DN=2.2`
- `MPC_Z_V_AUTO_DN=2.2`
- `MPC_LAND_ALT1=8.0`
- `MPC_LAND_ALT2=3.0`
- `MPC_LAND_SPEED=0.6`
- `MPC_LAND_CRWL=0.30`
- `NPFG_PERIOD=35.0`
- `NPFG_DAMPING=0.80`
- `NPFG_ROLL_TC=0.7`
- `NPFG_SW_DST_MLT=0.7`
- `NAV_LOITER_RAD=2000.0`
- `RTL_LOITER_RAD=2000.0`
- `CAL_BARO0_ID=6620172`
- `CAL_BARO0_PRIO=100`
- `CAL_BARO1_ID=6620428`
- `CAL_BARO1_PRIO=0`
- `EKF2_BARO_NOISE=1.0`
- `CAL_ACC0_PRIO=50`
- `ASPD_DO_CHECKS=1`
- `SYS_HAS_NUM_ASPD=0`
- `VT_ARSP_TRANS=46.0`
- `VT_F_TRANS_DUR=45.0`
- `VT_F_TR_OL_TM=55.0`
- `FW_T_ALT_TC=6.0`
- `FW_T_RLL2THR=12.0`
- `FW_T_STE_R_TC=2.0`
- `FW_T_THR_DAMPING=0.25`
- `FW_T_PTCH_DAMP=0.25`
- `FW_T_I_GAIN_PIT=0.20`
- `FW_T_CLMB_R_SP=2.0`
- `FW_T_SINK_R_SP=1.5`
- `FW_PSP_OFF=3.0`
- `FW_P_LIM_MAX=18.0`
- `FW_R_LIM=22.0`
- `FW_THR_TRIM=0.80`
- `FW_THR_SLEW_MAX=0.50`
- `WEIGHT_BASE=3120.0`
- `WEIGHT_GROSS=3120.0`
- `VT_B_TRANS_DUR=35.0`
- `VT_B_DEC_MSS=1.5`
- `VT_B_DEC_I=0.25`
- `VT_B_TRANS_RAMP=15.0`
- `RTL_RETURN_ALT=100`
- `RTL_DESCEND_ALT=80`
- `LNDMC_Z_VEL_MAX=0.25`
- `COM_DISARM_LAND=1.5`

Do not use the exact accelerometer offset values as a package pass/fail check.
PX4 can update them after a run; verify estimator behavior from ULog instead.

In the X-Plane `Log.txt`, confirm:

- `px4xplane: Version: v4.0.2`
- `px4xplane: No autoPropBrakes specified or parameter not found for configuration: Alia250`
- `px4xplane: Motor brakes configured for motors: 00000000`

The Alia lift-prop brake policy is generic and opt-in from `config.ini`: it
only applies when `autoPropBrakes` lists motor indices. v4.0.0 keeps Alia
lift-prop braking disabled by default because the accepted no-brake handoff
kept more recovery margin than the brake-window test. `feather`, `hard_lock`,
and `prop_separate` remain available for controlled A/B testing only.

If the first connection attempt times out, click `Connect to SITL` again after
PX4 is visibly waiting on `simulator_mavlink`. A first timeout usually means
PX4 and the plugin missed one TCP retry window; the retry is safe and should not
change the active airframe or parameters.

The `px4xplane/px4_airframes` folder in plugin releases contains reference
copies only. PX4 SITL reads the airframe from the PX4 repository branch under
`ROMFS/px4fmu_common/init.d-posix/airframes/`.

If a low-FPS startup produces `High Accelerometer Bias` or `vertical velocity
unstable`, do not tune from that run until the aircraft has been restarted with
the v4.0.0 airframe defaults and the warning state is rechecked. Keep the log:
that case is useful for validating frame-rate robustness, but it is not the
same evidence as a clean high-FPS Alia tuning flight.

During front transition, keep the aircraft on a long straight outbound segment
where practical. PX4 Standard VTOL transition should be judged by airspeed,
height loss, TECS pitch/throttle response, and the first fixed-wing altitude
capture, not by forcing an early tight turn before the wing-borne state is
settled.
