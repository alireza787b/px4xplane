# Report v13 - Alia Retest Regression Triage

Date: 2026-05-07

## Evidence Reviewed

- `/home/alireza/alia-sitl2/19_07_20.ulg`
- `/home/alireza/alia-sitl2/19_10_42.ulg`
- Previous successful reference: `/home/alireza/alia-sitl1/05_17_33.ulg`
- Local test packages and release artifacts for:
  - `px4xplane v3.4.5`
  - `XPlaneTruthCapture v0.1.6`
  - `PX4-Autopilot-Me px4xplane-sitl`

No X-Plane `Log.txt` or TruthCapture run data was present in `alia-sitl2` during
this investigation.

## Findings

### 1. TruthCapture Did Not Load Because the Local Windows Package Was Unsafe

The `/home/alireza/XPlaneTruthCapture-Windows-v0.1.6.zip` package was built
locally with MinGW. The included `win.xpl` imported:

- `libgcc_s_seh-1.dll`
- `libstdc++-6.dll`

Those DLLs were not included in the zip. X-Plane can fail plugin loading before
the menu is created in that case, which matches the reported symptom that
XPlaneTruthCapture did not appear in the menu.

Fix: `XPlaneTruthCapture v0.1.7` statically links the MinGW GCC runtime for
local Windows builds, so the provided local package does not depend on external
MinGW runtime DLLs.

### 2. The New ULogs Are Not a Clean Bad-Alia vs Good-Alia Pair

`19_07_20.ulg` reports:

- `SYS_AUTOSTART=5010`

That is not the Alia airframe (`5020`). It should not be used to judge Alia
tuning.

`19_10_42.ulg` reports:

- `SYS_AUTOSTART=5020`
- Key MC/VTOL/FW parameters matching the full successful
  `alia-sitl1/05_17_33.ulg`

It has only about 36 seconds of topic data in the uploaded file, so it is useful
for early takeoff comparison but not for full RTL/landing conclusions.

### 3. The Previous Param Sync Used the Wrong Source of Truth

The successful reference ULog is the authority for the current Alia baseline.
The plugin repo airframe file still contained unproven follow-up tuning values
for FW/TECS/NPFG/loiter behavior. Syncing that file into the PX4 fork treated
repo state as truth instead of the proven ULog baseline.

Fix: `px4xplane v3.4.6` restores the Alia FW/TECS/NPFG/loiter defaults to the
values observed in the full successful Alia ULog:

- `NPFG_PERIOD=45.0`
- `NPFG_DAMPING=0.7`
- `NAV_LOITER_RAD=2000.0`
- `RTL_LOITER_RAD=2000.0`
- `FW_R_LIM=30.0`
- `FW_T_ALT_TC=4.0`
- `FW_T_STE_R_TC=1.5`
- `FW_T_THR_DAMPING=0.12`
- `FW_T_PTCH_DAMP=0.14`
- `FW_T_I_GAIN_PIT=0.3`
- `FW_THR_TRIM=0.80`
- `MC_ORBIT_RAD_MAX=2000`

### 4. Parameter Persistence Can Mask Airframe File Changes

PX4 SITL imports saved parameters at startup. PX4 documentation distinguishes
`param load` from `param import`: `load` resets then applies a file, while
`import` merges values into the current state. PX4 airframe documentation also
notes `PARAM_DEFAULTS_VER` for forced airframe-default resets across updates.

References used for this decision:

- PX4 Parameters & Configurations:
  <https://docs.px4.io/main/en/advanced/parameters_and_configurations.html>
- PX4 Adding a Frame Configuration:
  <https://docs.px4.io/main/en/dev_airframes/adding_a_new_frame>

For the next comparison run, use the setup script `distclean` option once after
installing the new airframe file. This avoids saved `parameters.bson` state
masking the intended default values.

Manual copy is acceptable as a temporary workaround when `git pull` is not
available on the GPU PC, but it only changes the airframe script on disk. PX4
can still start from saved SITL parameter state unless defaults are re-applied,
so the manual-copy workflow must include a reset/distclean step before testing.

### 5. Airspeed Selector Warning

The CLI warning `Preflight Fail: Airspeed selector module down` appeared at
startup, but the later log line showed the airspeed selector publishing scale
updates. The successful reference run used `FW_USE_AIRSPD=1` and
`ASPD_DO_CHECKS=7`; this warning alone is not evidence of a flight-critical
regression. Keep it in the observation list and re-check after complete logs are
available.

## Fixes Prepared

- `XPlaneTruthCapture v0.1.7`
  - Static MinGW runtime link for local Windows packages.
  - Version bump and README note.
- `px4xplane v3.4.6`
  - Alia FW/TECS/NPFG/loiter values restored to the successful ULog baseline.
  - Alia test instructions updated to require parameter reset/distclean.
- `PX4-Autopilot-Me px4xplane-sitl`
  - Will be synced to the same v3.4.6 Alia airframe file.

## Next Test Requirements

Return these four artifacts after the retest:

- X-Plane `Log.txt`
- PX4 `.ulg`
- PX4 SITL CLI log
- XPlaneTruthCapture run folder/zip

If TruthCapture still does not show in the menu, X-Plane `Log.txt` is required;
it should contain the plugin loader reason.
