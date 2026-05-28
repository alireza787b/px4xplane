# Report v72 - TB2 Estimator and Landing Closure

Date: 2026-05-28
Package target: `v3.4.60`

## Evidence Reviewed

- `/home/alireza/tb2.zip`
- X-Plane `Log.txt`
- PX4 ULog `09_00_55.ulg`
- XPlaneTruthCapture run `20260528-085607Z`
- Independent read-only review from a second agent

## Root Causes

### Startup Config Warning

The TB2 config warning was a false positive. At plugin startup X-Plane returned
only the simulator root path, `H:\X-Plane 12\`, while the user aircraft `.acf`
had not been populated yet. The real aircraft load appeared later as:

```text
Aircraft/Bayraktar_tb2/Bayraktar_tb2.acf
```

The same run later reloaded the config and reported `Config: OK`, so the TB2
`aircraftMatch = Bayraktar_tb2,TB2` token set was correct.

### High Accelerometer Bias

Two issues overlapped:

- The ULog still had stale/manual PX4 values: `EKF2_ABL_LIM=2.0`,
  `EKF2_GPS_DELAY=10`, `EKF2_MULTI_IMU=3`, and `IMU_INTEG_RATE=250` instead of
  the current airframe defaults. Those values must be reset before judging
  estimator health.
- TruthCapture showed uncommanded ground creep up to about `5.1 m/s` before the
  detected takeoff roll. The bridge stationary-ground contract was still too
  narrow for that idle/contact motion, so PX4 could see real X-Plane ground
  acceleration before PX4 commanded throttle.

The in-flight estimator bias also showed a repeatable TB2 simulated
accelerometer offset, especially Z near `-0.77 m/s^2`, so TB2 now follows the
accepted Cessna approach: source-control simulated `CAL_ACC0_*OFF` offsets while
keeping `EKF2_ABL_LIM=0.8`.

### Approach Overspeed and Flare

PX4 commanded the intended `FW_LND_AIRSPD=28 m/s`, but the aircraft actually
flew the final approach around `45-48 m/s`. During approach TECS still commanded
roughly `60%` throttle, so the aircraft arrived with too much energy. During
flare the pitch command reached the old high flare limit, ballooned, then the
aircraft dropped onto the runway.

PX4 fixed-wing landing documentation identifies `FW_LND_ANG`,
`FW_LND_AIRSPD`, `FW_LND_FLALT`, `FW_LND_FL_PMIN`, `FW_LND_FL_PMAX`, and
`FW_LND_THRTC_SC` as the key landing/flare parameters. PX4 parameter reference
documents `FW_LND_THRTC_SC` as the landing multiplier that makes TECS react
faster during landing when set below `1.0`.

References:

- https://docs.px4.io/main/en/flight_modes_fw/land
- https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- https://docs.px4.io/v1.15/en/advanced_config/parameter_reference

## Implemented Changes

### px4xplane Bridge

- Deferred `aircraftMatch` warnings until X-Plane reports an actual `.acf`
  model path. This removes the startup banner false warning without weakening
  real mismatch detection after aircraft load/config reload.
- Increased the stationary-ground no-command speed hysteresis from
  `0.75/1.20 m/s` to `6.0/8.0 m/s`. This catches uncommanded idle creep and
  contact-physics motion, while still releasing immediately on mapped PX4 motor
  command.
- Updated the truth replay tool and unit tests to match the bridge ground
  contract.

### TB2 PX4 Airframe

- Added TB2 simulated accelerometer offsets:
  - `CAL_ACC0_XOFF=-0.030`
  - `CAL_ACC0_YOFF=0.010`
  - `CAL_ACC0_ZOFF=-0.770`
- Kept `EKF2_ABL_LIM=0.8`; the fix is calibration plus ground-contract
  robustness, not a large hidden bias limit.
- Reduced TB2 cruise/landing energy:
  - `FW_THR_TRIM: 0.45 -> 0.36`
  - `FW_THR_SLEW_MAX: 0.35 -> 0.25`
  - `FW_T_ALT_TC: 8.0 -> 7.0`
  - `FW_T_SPDWEIGHT: 0.9 -> 0.8`
- Changed landing flare/throttle behavior:
  - `FW_LND_THRTC_SC: 0.7 -> 0.3`
  - `FW_LND_FL_PMIN: 10 -> 8 deg`
  - `FW_LND_FL_PMAX: 20 -> 14 deg`
  - `FW_LND_FL_SINK: 0.25 -> 0.35 m/s`
  - `FW_LND_TD_OFF: 5 -> 4 m`

No TB2 `.acf` change was made in this slice. The returned log showed enough
PX4 throttle authority and flap deployment evidence to fix this first in the
airframe/bridge contract without changing aircraft physics.

## Verification

Commands run:

```text
python3 -m unittest tests/test_replay_truth_capture.py
ctest --test-dir build-offline-tests --output-on-failure
cmake --build build-offline-tests -- -j$(nproc)
cmake --build build-win-mingw -- -j$(nproc)
```

Results:

- Replay tests: 6/6 passed.
- CTest: 5/5 passed.
- Linux package build: passed.
- Windows package build: passed.
- Replay check on `tb2.zip`: the stationary-ground contract stays active
  through the initial idle creep and releases when PX4 motor command appears.

## Test Instructions

Use `v3.4.60` and pull the PX4 `px4xplane-sitl` branch. Run a clean PX4 state:

```bash
make px4_sitl_default distclean
rm -f build/px4_sitl_default/rootfs/parameters.bson \
      build/px4_sitl_default/rootfs/parameters_backup.bson
```

Then run `make px4_sitl_default xplane_tb2`.

Before judging the run, confirm the ULog has the v3.4.60 TB2 defaults:

```text
SYS_AUTOSTART=5002
EKF2_ABL_LIM=0.8
CAL_ACC0_XOFF=-0.030
CAL_ACC0_YOFF=0.010
CAL_ACC0_ZOFF=-0.770
EKF2_GPS_DELAY=0.0
EKF2_MULTI_IMU=1
IMU_INTEG_RATE=200
PWM_MAIN_FUNC6=440
PWM_MAIN_FUNC7=205
PWM_MAIN_FUNC8=206
FW_THR_TRIM=0.36
FW_LND_THRTC_SC=0.3
```

Use a long final approach. If PX4 reports an infeasible fixed-wing landing
slope, increase the approach distance or lower the landing entrance altitude
instead of making the slope steeper.

## Expected Result

- No false TB2 config warning after the TB2 `.acf` has loaded.
- No high-accelerometer-bias warning while stationary or during idle creep.
- Runway takeoff/centerline remains as good as `tb2.zip`.
- Cruise/lateral tracking remains clean.
- Final approach tracks closer to `28 m/s`; it should not stay at `45-48 m/s`.
- Flare should avoid the previous balloon-then-drop behavior.
