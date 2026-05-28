# Report v73 - TB3 Autoland Energy and Rollout Closure

Date: 2026-05-28
Package target: `v3.4.61`

## Evidence Reviewed

- `/home/alireza/tb3.zip`
- PX4 ULog `15_04_54.ulg`
- X-Plane `Log.txt`
- XPlaneTruthCapture run `20260528-150249Z`

## TB3 Findings

The v3.4.60 bridge loaded correctly. X-Plane reported:

```text
px4xplane: Version: v3.4.60 (TB2 estimator and landing closure)
Loaded configuration: TB2
Config: OK
```

TruthCapture showed a clean simulator run: `52108` frames, no dropped rows, no
pause rows, no sim-time resets, mean callback rate about `117.7 Hz`, and a
maximum frame period near `0.05 s`. The earlier startup config warning was not
present after the TB2 `.acf` was available.

The ULog did not contain a high accelerometer bias warning. It did, however,
still contain stale saved SITL defaults for several estimator parameters:

```text
EKF2_GPS_DELAY = 10.0   expected 0.0
EKF2_MULTI_IMU = 3      expected 1
IMU_INTEG_RATE = 250    expected 200
```

That confirmed a workflow bug: even when the airframe file is updated, PX4 can
keep old saved values in `build/px4_sitl_default/rootfs/parameters.bson`.

## Approach Energy Root Cause

The observed final approach speed was not caused by a high landing airspeed
target. PX4 was already using:

```text
FW_AIRSPD_TRIM = 36 m/s
FW_LND_AIRSPD = 28 m/s
```

The TB3 final approach still carried roughly mid-30s m/s true airspeed. ULog
TECS evidence showed the landing pre-flare segment was commanding the intended
`28 m/s` airspeed, but actual true airspeed stayed around `33-35 m/s`, while
throttle remained high enough to keep extra energy in the aircraft. The previous
`FW_LND_THRTC_SC=0.3` made the low-height landing altitude time constant very
short, so TECS chased glide-slope errors with aggressive throttle/pitch changes.

PX4 fixed-wing landing documentation describes the mission landing sequence as
descent orbit, landing approach, flare, and rollout. It lists `FW_LND_AIRSPD`,
`FW_FLAPS_LND_SCL`, and `FW_LND_THRTC_SC` as landing-approach parameters. PX4's
fixed-wing tuning guide also states that TECS coordinates throttle and pitch to
control airspeed and altitude, and that tuning should be verified from logs.

References:

- https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- https://docs.px4.io/v1.15/en/flight_modes_fw/mission
- https://baykartech.com/en/uav/bayraktar-tb2/

Baykar's public TB2 page lists a 90-110 KTAS travel/max speed band. Keeping the
simulated trim speed at `36 m/s` remains conservative for this model; the next
test should focus on the final energy path before any cruise-speed increase.

## Rollout Root Cause

TruthCapture marked ground contact at frame `46430` and stopped-after-landing
at frame `50811`, about `36.9 s` later in sim time. During the first rollout
window the aircraft was still moving around `23 m/s` median groundspeed.

PX4 declared landed and disarmed much earlier than the aircraft had fully
settled. With v3.4.60 using `LNDFW_AIRSPD_MAX=30`, `LNDFW_VEL_XY_MAX=15`, and
the default `COM_DISARM_LAND=2`, PX4 could stop active runway steering while the
aircraft was still rolling at a meaningful speed. That matches the visual
observation: touchdown happened, then the aircraft became unstable and drifted
out during rollout.

## Implemented Changes

### PX4 Setup Workflow

`setup/setup_px4_sitl.sh` now fingerprints the selected PX4 airframe file. If
the selected airframe or file hash changes since the previous run, it removes
only:

```text
build/px4_sitl_default/rootfs/parameters.bson
build/px4_sitl_default/rootfs/parameters_backup.bson
```

This forces the current `set-default` values to load on the next SITL start
without deleting the whole PX4 checkout. `make distclean` in the setup script
also calls the same reset helper.

### TB2 PX4 Airframe

The TB2 airframe was updated in both px4xplane package params and the PX4 fork:

```text
FW_T_ALT_TC       7.0  -> 8.0
FW_T_HRATE_FF     0.3  -> 0.15
FW_T_SPDWEIGHT    0.8  -> 1.0
FW_T_CLMB_MAX     3.0  -> 2.5
FW_T_CLMB_R_SP    2.0  -> 1.5
FW_T_SINK_MAX     2.5  -> 2.0
FW_T_SINK_R_SP    1.5  -> 1.0
FW_T_VERT_ACC     2.2  -> 1.5
FW_T_THR_DAMPING  0.18 -> 0.25
FW_T_PTCH_DAMP    0.20 -> 0.30
FW_T_I_GAIN_PIT   0.25 -> 0.15
FW_THR_TRIM       0.36 -> 0.32
FW_THR_MIN        0.08 -> 0.05
FW_THR_SLEW_MAX   0.25 -> 0.30
FW_LND_ANG        3.5  -> 3.0
FW_LND_THRTC_SC   0.3  -> 0.7
FW_FLAPS_LND_SCL  0.75 -> 1.0
LNDFW_AIRSPD_MAX  30   -> 12
LNDFW_VEL_XY_MAX  15   -> 6
LNDFW_VEL_Z_MAX   added as 1.5
LNDFW_TRIG_TIME   added as 2.0
COM_DISARM_LAND   added as 4.0
NAV_FW_ALT_RAD    35   -> 25
```

The intent is:

- preserve the accepted takeoff, cruise, and lateral tracking;
- keep the same `28 m/s` final target;
- make landing altitude tracking less aggressive near ground;
- reduce trim/min throttle energy;
- use full landing flaps;
- keep PX4 steering/armed state until rollout is slower.

## Next Test Instructions

Use the `v3.4.61` package and pull the PX4 `px4xplane-sitl` branch. Running
through the setup script should automatically reset saved SITL params if the TB2
airframe file hash changed.

If running PX4 manually instead of the setup script, reset once before the next
test:

```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson \
      build/px4_sitl_default/rootfs/parameters_backup.bson
make px4_sitl_default xplane_tb2
```

Before judging the next flight, confirm these initial ULog values:

```text
EKF2_GPS_DELAY=0.0
EKF2_MULTI_IMU=1
IMU_INTEG_RATE=200
FW_AIRSPD_TRIM=36.0
FW_LND_AIRSPD=28.0
FW_THR_TRIM=0.32
FW_LND_THRTC_SC=0.7
FW_FLAPS_LND_SCL=1.0
LNDFW_AIRSPD_MAX=12.0
LNDFW_VEL_XY_MAX=6.0
COM_DISARM_LAND=4.0
```

Expected result: takeoff and cruise should stay close to TB3, final should
bleed down toward `28 m/s` instead of staying in the mid-30s, flare should be
less throttle-chasing, and rollout should remain under PX4 control longer before
disarm.
