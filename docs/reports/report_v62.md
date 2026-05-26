# Report v62 - qtail26 Closure Candidate and v3.4.50

Date: 2026-05-26

Package target: `v3.4.50`

## Summary

`qtail26.zip` is the first fully accepted QuadTailsitter mission baseline. The
aircraft completed takeoff, front transition, fixed-wing mission flight,
return, back-transition, multicopter descent, landing, disarm, and PX4 log
close. The previous bridge regressions did not reappear: no baro-stale,
vertical-velocity-unstable, accelerometer-bias, pause/FPS, or sensor-reset
warnings were present in the accepted run.

This slice keeps the qtail26 aircraft physics, bridge sensor contract,
body-axis pitot, rotor cant, and back-transition timing as the baseline. The
only flight-tuning changes are conservative fixed-wing energy polish values.
It also adds source-controlled X-Plane presentation assets requested for the
demo: aircraft icons and quick-look camera presets.

## qtail26 Evidence

- TruthCapture recorded `45,723` frames, `0` dropped rows, `0` sim-time resets,
  about `88 Hz` mean callback rate, and a maximum frame period of about
  `50 ms`.
- X-Plane loaded `px4xplane v3.4.49`, `Config Name: QuadTailsitter`, the
  QuadTailsitter ACF, and the body-axis virtual pitot with `pitotAxisBody=-Z`.
- The PX4 ULog matched the intended qtail26 airframe defaults. The only
  expected parameter checker mismatch was `SYS_AUTOSTART`, which is selected by
  the PX4 launch target rather than by the airframe file.
- Fixed-wing flight was stable enough for the acceptance gate: truth TAS p50
  was about `28.7 m/s`, altitude held near `100 m`, and mean motor command p50
  was about `0.10`.
- Back-transition was safe and produced no failure-detector warnings, but the
  aircraft still converted forward-speed energy into a recovery climb to about
  `202 m AGL` before descending in multicopter mode.
- Landing and disarm completed normally.

## Design Check

The current aircraft sizing remains plausible and should not be redesigned in
this slice:

- loaded truth mass: about `5.44 kg`
- empty mass: about `4.99 kg`
- wing area estimate: about `0.509 m^2`
- loaded wing loading: about `10.7 kg/m^2`
- stall estimate: about `12 m/s`
- qtail26 fixed-wing TAS median: about `28.7 m/s`

The flight envelope still has more than `2x` stall margin in the current demo
profile. Low fixed-wing throttle is a model characteristic to keep watching,
but qtail26 did not justify mixing another ACF physics change into the closure
candidate.

## Research Basis

- PX4 fixed-wing position tuning separates TECS altitude/airspeed behavior from
  NPFG horizontal path tracking. That supports small airspeed/throttle trim
  changes for energy polish while avoiding another lateral guidance rewrite:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning defines `VT_B_TRANS_DUR` as the duration
  limit and `VT_B_DEC_MSS` as the expected deceleration used for VTOL Land
  geometry:
  https://docs.px4.io/v1.11/en/config_vtol/vtol_back_transition_tuning.html
- X-Plane aircraft-selection icons use the aircraft-local `*_icon11.png` and
  `*_icon11_thumb.png` assets. The normal icon is `800x450`; the thumbnail is
  `174x107`:
  https://developer.x-plane.com/version/x-plane-11/
- Plane Maker is the correct tool for polished cockpit/panel authoring. The
  current package therefore avoids brittle hand-edits to panel records and
  leaves a real eVTOL panel/MFD as a later Plane Maker slice:
  https://developer.x-plane.com/manuals/planemaker/index.html
- X-Plane quick-look views can be saved and recalled through native view
  commands. The package now ships aircraft-local quick-look presets so the demo
  has immediate FPV, belly, and engineering views:
  https://x-plane.com/manuals/desktop/11/index.html

## v3.4.50 Changes

- Increased QuadTailsitter vertical velocity damping:
  `MPC_Z_VEL_D_ACC 0.5 -> 0.6`.
- Reduced fixed-wing cruise energy slightly:
  `FW_AIRSPD_TRIM 28 -> 27`, `FW_AIRSPD_MAX 38 -> 36`, and
  `FW_THR_TRIM 0.15 -> 0.14`.
- Kept qtail26-proven back-transition timing unchanged:
  `VT_B_TRANS_DUR=5.0`, `VT_B_DEC_MSS=1.0`.
- Added `QuadTailsitter_icon11.png` and `QuadTailsitter_icon11_thumb.png`.
- Added quick-look presets in `QuadTailsitter_prefs.txt`:
  nose/FPV, belly/down-looking, and rear engineering chase.
- Added `aircraft/QuadTailsitter/README.md` documenting the quick-look/icon
  assets and deferring the richer cockpit panel to a Plane Maker-authored slice.
- Updated the QuadTailsitter X-Plane 12 test card for qtail26 and v3.4.50.

## Independent Review Result

The independent review agreed with keeping the accepted qtail26 baseline. It
specifically rejected shipping an untested `VT_B_TRANS_DUR=5.5` and
`VT_B_DEC_MSS=0.8` pair because qtail26 showed the back-transition climb is
mostly stored forward-speed recovery, and a longer duration may increase the
energy available at MC takeover. Those two exploratory values were removed
before packaging.

## Next Test

Use v3.4.50 for one closure run before moving to the next airframe:

- Force a clean PX4 parameter load (`make distclean` or remove SITL
  `parameters.bson` / `parameters_backup.bson`).
- Confirm the ULog loads `FW_AIRSPD_TRIM=27`, `FW_AIRSPD_MAX=36`,
  `FW_THR_TRIM=0.14`, `MPC_Z_VEL_D_ACC=0.6`, `VT_B_TRANS_DUR=5.0`, and
  `VT_B_DEC_MSS=1.0`.
- Use high-margin VTOL Land/back-transition altitude.
- Validate that the accepted qtail26 path behavior remains intact and that the
  recovery climb is no worse.
- Send ULog, PX4 terminal output, X-Plane `Log.txt`, and TruthCapture folder.
