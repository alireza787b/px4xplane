# Report v71 - TB2 First-Test Triage and Config Guard

## Evidence Reviewed

- `/home/alireza/tb1.zip`
- X-Plane `Log.txt`
- PX4 ULog `04_26_55.ulg`
- XPlaneTruthCapture run `20260528-042416Z`

## Root Cause Status

`tb1.zip` is useful but not a clean v3.4.58 TB2 validation:

- X-Plane loaded `px4xplane` `v3.4.57`.
- The plugin initially parsed `Cessna172` while the loaded aircraft was
  `Bayraktar_tb2.acf`.
- You then selected `TB2` from the plugin menu before connecting PX4, so the
  flight did use the TB2 bridge section, but not the intended v3.4.58 package.
- PX4 ran `SYS_AUTOSTART=5002`, but the ULog showed stale/manual values such as
  `EKF2_ABL_LIM=2.0` and `IMU_INTEG_RATE=250`, so estimator/landing conclusions
  are contaminated until a clean parameter reset is confirmed.

## What The Log Still Proved

- PX4 commanded runway wheel steering and landing flaps.
- XPlaneTruthCapture showed physical flaps reached roughly the expected landing
  deflection range, but the setup was not the final TB2 package.
- Vertical energy management was too aggressive: TECS allowed large climb/sink
  rates, the mission included a steep landing geometry, and the touchdown did
  not show a strong flare margin.
- The pre-PX4 “slight movement” was not supported by truth data: prearm ground
  speed was effectively zero. No ACF idle-thrust change is justified from this
  evidence alone.

## Changes Applied

### Generic Setup Guard

Added `aircraftMatch` as an optional per-airframe config key:

```ini
aircraftMatch = Bayraktar_tb2,TB2
```

The bridge checks the loaded X-Plane user aircraft filename/path and warns in
`Log.txt` and the connection HUD if the active config appears mismatched. This
would have made the initial Cessna-active/TB2-aircraft state visible before
connecting PX4.

This is not hardcoded to TB2 and does not auto-switch configs. It is a warning
guard only.

### TB2 Parameter Tune

Kept the TB2 bridge mapping baseline and made conservative PX4 airframe changes:

- stronger runway wheel steering: `FW_WR_P=1.2`, `FW_WR_I=0.18`,
  `FW_WR_FF=0.60`
- slower vertical TECS response: `FW_T_CLMB_MAX=3.0`,
  `FW_T_SINK_MAX=2.5`, `FW_T_SINK_R_SP=1.5`, `FW_T_VERT_ACC=2.2`
- less throttle pumping in turns: `FW_T_RLL2THR=6.0`,
  `FW_T_THR_DAMPING=0.18`, `FW_T_PTCH_DAMP=0.20`
- runway takeoff target updated to `FW_TKO_AIRSPD=32.0`
- landing made shallower and flare earlier: `FW_LND_ANG=3.5`,
  `FW_LND_FLALT=10.0`, `FW_LND_FL_PMIN=10.0`,
  `FW_LND_FL_PMAX=20.0`, `FW_LND_TD_OFF=5.0`

Landing airspeed remains `28 m/s` for this slice. The returned log was not clean
enough to justify lowering stall margin yet.

## Next Test Gate

Before judging the next TB2 flight:

1. X-Plane `Log.txt` must show `px4xplane: Version: v3.4.59`.
2. X-Plane `Log.txt` must show active `TB2` before connecting PX4.
3. HUD/Log must not show an aircraft/config warning.
4. PX4 must be run after `make px4_sitl_default distclean`.
5. ULog initial parameters must match:
   - `SYS_AUTOSTART=5002`
   - `EKF2_ABL_LIM=0.8`
   - `IMU_INTEG_RATE=200`
   - `EKF2_GPS_DELAY=0.0`
   - `EKF2_MULTI_IMU=1`
   - `PWM_MAIN_FUNC6=440`
   - `PWM_MAIN_FUNC7=205`
   - `PWM_MAIN_FUNC8=206`

If any of those are stale, treat the run as setup evidence, not tuning evidence.
