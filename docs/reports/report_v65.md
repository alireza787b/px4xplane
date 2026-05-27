# Report v65 - Config Editor UX Fixes and Cessna 172 Baseline

Package target: `v3.4.53`

## Scope

This slice closes the config-editor feedback from the v3.4.52 package and
prepares the first Cessna 172 validation baseline.

No accepted Alia, Ehang, or QuadTailsitter PX4 flight-control tuning is changed
in this slice.

## Decisions

The runtime source of truth remains:

```text
px4xplane/64/config.ini
```

`config_schema.json` remains metadata only. It documents field types, ranges,
validation, and editor help; the plugin does not read it at runtime. The editor
now tells the user to load `config.ini` if browser local-file policy blocks
auto-loading, and it rejects JSON in the runtime-config file picker instead of
appearing to do nothing.

Alia remains the packaged default active airframe:

```ini
config_name = Alia250
```

For Cessna testing, change it deliberately to:

```ini
config_name = Cessna172
```

## Evidence and References

- PX4 fixed-wing takeoff documentation says runway takeoff is enabled with
  `RWTO_TKOFF=1`, requires the wheel controller (`FW_W_EN`), ramps throttle with
  `RWTO_RAMP_TIME`, rotates at `RWTO_ROT_AIRSPD`, then climbs using
  `FW_T_CLMB_MAX` toward `MIS_TAKEOFF_ALT`:
  https://docs.px4.io/main/en/flight_modes_fw/takeoff.html
- PX4 fixed-wing position tuning documentation describes the TECS/NPFG split:
  TECS controls altitude/airspeed through pitch and throttle, while NPFG handles
  horizontal heading/path tracking:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 fixed-wing landing documentation identifies `FW_LND_ANG`,
  `FW_LND_AIRSPD`, `FW_LND_FLALT`, `FW_LND_FL_PMIN`, and `FW_LND_FL_PMAX` as
  the landing slope/flare controls:
  https://docs.px4.io/main/en/flight_modes_fw/land
- Textron/Cessna Skyhawk specifications used for the initial Cessna target:
  2,550 lb MTOW, 36 ft 1 in wingspan, 124 ktas max cruise, 48 kcas stall,
  730 fpm maximum climb, 180 hp IO-360-L2A:
  https://txtav.com/sitecore/content/cessnaweb/home/piston/cessna-skyhawk?sc_lang=en
- X-Plane exposes several dataref types, but px4xplane actuator output remains
  intentionally limited to `float` and `floatArray` because PX4 actuator outputs
  are normalized floating-point setpoints:
  https://developer.x-plane.com/sdk/XPLMDataAccess/

## Config Editor Changes

- Empty state is calmer: no preview wall and no validation alarm before a config
  is loaded.
- Runtime config picker is labeled `Load config.ini`; schema picker is labeled
  advanced metadata.
- Loading JSON in the runtime picker now produces a direct instruction to load
  `px4xplane/64/config.ini`.
- Description column was removed. Field descriptions are available from the `?`
  tooltip and reload-policy badge.
- Repeated generic Docs links were removed. Only channel mappings and camera
  views keep specific docs links.
- Airframe list shows the active airframe first and collapses inactive
  airframes.
- The selected airframe editor now presents setup fields, actuator mappings,
  then camera views.
- Actuator index and output range fields use structured controls rather than a
  raw `[0]` / `[-1 1]` string.

## Cessna 172 PX4 Baseline

The Cessna file was sanitized and synced to the PX4 PR branch:

```text
config/px4_params/5001_xplane_cessna172
PX4-Autopilot-Me/ROMFS/px4fmu_common/init.d-posix/airframes/5001_xplane_cessna172
```

Key changes:

- Removed stale overclaiming comments and old January 2025 tuning notes.
- Added simulated magnetometer enable (`SENS_EN_MAGSIM=1`, `SYS_HAS_MAG=1`).
- Brought IMU rate caps to `200 Hz`, matching the current bridge target.
- Replaced stale `EKF2_ABL_LIM=2.5` with current metadata maximum `0.8`.
- Replaced stale `NAV_ACC_RAD=300` with current metadata-safe `180`.
- Changed virtual-pitot checks to the accepted simulator policy:
  `ASPD_DO_CHECKS=1`, `ASPD_FALLBACK=1`, `SYS_HAS_NUM_ASPD=0`.
- Added runway takeoff defaults: `RWTO_TKOFF=1`, `FW_W_EN=1`,
  `RWTO_ROT_AIRSPD=28`, `FW_TKO_AIRSPD=38`, and a less aggressive
  `FW_TKO_PITCH_MIN=10`.
- Added initial fixed-wing landing settings for mission landing validation:
  `FW_LND_ANG=5`, `FW_LND_AIRSPD=36`, `FW_LND_FLALT=5`,
  `FW_LND_FL_PMIN=4`, `FW_LND_FL_PMAX=18`.
- Cleaned RTL altitude ordering: `RTL_RETURN_ALT=120`,
  `RTL_DESCEND_ALT=80`.
- Set mass to the Cessna reference MTOW: `WEIGHT_BASE/GROSS=1157`.

## Test Card

Added and packaged:

```text
docs/CESSNA172_XPLANE12_TEST.md
```

The first Cessna test should be a runway mission:

1. Align the Cessna on a paved runway.
2. Set `config_name = Cessna172`.
3. Reset PX4 SITL parameters once after replacing the PX4 airframe file.
4. Run `make px4_sitl_default xplane_cessna172`.
5. Execute a mission with runway takeoff, broad waypoints/loiter, and fixed-wing
   landing pattern.

## Review Notes

The Cessna defaults are intentionally conservative. We are not copying the
QuadTailsitter or Alia dynamics into a fixed-wing manned aircraft; we are only
migrating the proven simulator sensor contract and then applying Cessna-specific
speeds, mass, runway takeoff, landing, and guidance defaults.

The first returned Cessna log should be used to tune:

- runway wheel controller behavior
- TECS throttle/pitch damping
- NPFG turn smoothness and loiter radius
- landing flare and rollout/disarm timing

## Verification

Executed:

```bash
python3 tools/validate_config.py config/config.ini
node --check docs/assets/config-editor.js
node tests/test_config_editor.js
ctest --test-dir build --output-on-failure
cmake --build build --target px4xplane --parallel 2
cmake --build build-win-mingw --target px4xplane --parallel 2
```

Result: all editor/config tests and CTest cases passed. Linux and MinGW Windows
plugin builds completed. Compiler output still contains existing third-party
MAVLink packed-member warnings and older unused-parameter warnings, but no build
failure.

Also validated the Cessna airframe against PX4 parameter metadata. The old
`EKF2_ABL_LIM=2.5` and `NAV_ACC_RAD=300` out-of-bounds values did not return.

Packaged bundle:

```text
/home/alireza/px4xplane_v3.4.53_config_editor_cessna172_baseline_bundle.zip
```
