# Report v74 - PX4 Main Param Compatibility and Alia Startup Triage

Date: 2026-05-29

Package target: `v3.4.62`

## Evidence Reviewed

- User terminal paste from the failed startup.
- `/home/alireza/newAlia1.zip`:
  - `16_51_08.ulg`
  - X-Plane `Log.txt`
  - XPlaneTruthCapture run `20260529-165354Z`

## Findings

1. The pasted terminal and the uploaded ULog do not describe the same PX4
   startup.
   - Terminal paste: `SYS_AUTOSTART=5001`, which is Cessna 172.
   - Uploaded ULog: `SYS_AUTOSTART=5020`, `VT_TYPE=2`, `MAV_TYPE=22`, which is
     Alia VTOL.
   - X-Plane `Log.txt` and TruthCapture both show the Alia aircraft/config.

2. Current PX4 main rejected stale parameters from the X-Plane airframe files.
   - Visible errors: `EKF2_GPS_DELAY` and `EKF2_AGP_CTRL` were not found.
   - Static validation found additional removed params in active X-Plane
     airframes: `MPC_USE_HTE`, `LNDMC_TRIG_TIME`, and `MC_AT_START`.

3. The uploaded Alia ULog kept `SENS_GPS0_DELAY=10` and `SENS_GPS1_DELAY=10`.
   That happened because the old `EKF2_GPS_DELAY=0.0` line no longer maps when
   used in an airframe script. The corrected airframes now set both current
   sensor delay params directly to `0`.

4. The large logger warning block is separate from the airframe mismatch.
   Current PX4 main tries to subscribe many logger topics in full logging mode.
   The warnings are noisy, but the root startup defects in this slice were the
   wrong target evidence and stale parameters.

5. X-Plane `Error sending data: 10053` entries happened after PX4 disconnected
   while the plugin was still trying to send. This is disconnect noise, not the
   reason Alia failed to arm.

## Changes

- Updated all five packaged PX4 airframe reference files:
  - `5001_xplane_cessna172`
  - `5002_xplane_tb2`
  - `5010_xplane_ehang184`
  - `5020_xplane_alia250`
  - `5021_xplane_qtailsitter`
- Replaced `EKF2_GPS_DELAY=0.0` with:
  - `SENS_GPS0_DELAY=0`
  - `SENS_GPS1_DELAY=0`
- Removed obsolete defaults:
  - `EKF2_AGP_CTRL`
  - `MPC_USE_HTE`
  - `LNDMC_TRIG_TIME`
  - `MC_AT_START`
- Added launcher visibility:
  - selected target
  - selected `SYS_AUTOSTART`
  - selected PX4 airframe file
  - explicit reminder to match X-Plane aircraft/config before arming
- Updated current test-card docs and package index to `v3.4.62`.

## Verification

- PX4 build: `make px4_sitl_default` passed.
- PX4 startup smoke check:
  - `timeout 18s make px4_sitl_default xplane_alia250`
  - confirmed startup printed `SYS_AUTOSTART=5020`
  - no `Parameter ... not found` messages before simulator wait
- Static parameter validation against generated PX4 metadata:
  - all five `50xx_xplane_*` airframes passed with no unknown params.
- px4xplane checks:
  - `bash -n setup/setup_px4_sitl.sh`
  - `ctest --test-dir build-offline-tests --output-on-failure`
  - all 5 offline tests passed.

## Next Retest

For the next Alia startup, the PX4 terminal must show:

```text
Launching PX4 with X-Plane airframe (SYS_AUTOSTART=5020)
env SYS_AUTOSTART: 5020
```

If it shows `SYS_AUTOSTART=5001`, stop immediately because that is the Cessna
PX4 target.
