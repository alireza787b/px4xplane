# Report v63 - qtail27 Manual RTL and Config-Driven Camera Controls

Date: 2026-05-27

Package target: `v3.4.51`

## Scope

This slice reviews `/home/alireza/qtail27.zip`, fixes the manual RTL
back-transition crash path, and replaces the unreliable keyboard-dependent
quick-look workflow with config-driven plugin camera views.

## Evidence

qtail27 loaded the intended setup:

- X-Plane loaded px4xplane `v3.4.50`, `Config Name: QuadTailsitter`, and the
  packaged `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- Bridge config used the intended body-axis virtual pitot:
  `Airspeed source=body_axis pitotAxisBody=-Z`.
- `tools/check_px4_airframe_params.py` matched all checked qtail parameters.
- TruthCapture wrote `83,869` frames, with `0` dropped rows and `0` sim-time
  resets. Median frame-period rate was about `88.7 Hz`.

The crash sequence was specific:

- Manual RTL started at `t=541.974 s`: `RTL: start return at 126 m (100 m above destination)`.
- During FW RTL before back-transition, relative altitude descended from about
  `94.7 m` to about `71.8 m`. The position setpoint altitude was about
  `100.6 m` absolute, which is about `75 m` AGL for this run.
- Auto back-transition started at `t=603.962 s` near `76.5 m AGL`.
- During back-transition, horizontal speed increased from about `27 m/s` to
  `49 m/s`, motor mean rose from about `0.18` to `0.90`, and vertical descent
  reached about `10.3 m/s`.
- After MC handoff, altitude margin was gone: by `t=612 s` the vehicle was only
  about `12 m AGL`, descending about `18 m/s`.
- Roll failure appeared at `t=610.003 s`, followed by invalid MC setpoints and
  blind-land failsafe.

## Root Cause

The accepted qtail26/qtail27 flight-control tune was not missing from the test.
The unsafe condition was the old manual RTL descent altitude: `RTL_DESCEND_ALT=75`.
PX4 return behavior uses `RTL_RETURN_ALT` and `RTL_DESCEND_ALT` to choose the
return/descent altitude, and VTOL back-transition tuning guidance explicitly
warns that back-transition can glide/drift while altitude rather than position
is controlled. Sources checked:

- PX4 Return Mode: https://docs.px4.io/main/en/flight_modes/return
- PX4 VTOL Back-transition Tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning

qtail27 shows this particular tailsitter needs more altitude margin before
manual RTL auto back-transition. Changing the accepted ACF, body-axis pitot, or
back-transition duration would be a broader intervention than the evidence
justifies.

## Changes

QuadTailsitter PX4 defaults:

- `RTL_RETURN_ALT: 100 -> 180`
- `RTL_DESCEND_ALT: 75 -> 150`

px4xplane plugin:

- Added a `Camera Views` submenu under `PX4 X-Plane`.
- Added one reusable airframe-level `cameraViews` config key instead of
  hardcoding cameras for QuadTailsitter. Format:
  `Label|forward_m|right_m|up_m|pitch_offset_deg|heading_offset_deg|roll_offset_deg|zoom`
- Added `cameraViews` to `config/config_schema.json`, `tools/validate_config.py`,
  and the static config editor as a textarea with validation. This keeps normal
  users on the menu path while allowing advanced users to define cameras per
  airframe without rebuilding the plugin.
- Added generic bindable commands:
  - `px4xplane/camera/view_1`
  - `px4xplane/camera/view_2`
  - `px4xplane/camera/view_3`
  - up to `px4xplane/camera/view_8`
  - `px4xplane/camera/release`
- Implemented the views with X-Plane's `XPLMCamera` API using per-frame camera
  callbacks. X-Plane documents this as the plugin API for custom dynamic views:
  https://developer.x-plane.com/sdk/XPLMCamera/
- Menu entries execute commands with `XPLMAppendMenuItemWithCommand`, which
  X-Plane documents as command-backed menu items that users can bind to keys:
  https://developer.x-plane.com/sdk/XPLMAppendMenuItemWithCommand/

Camera orientation policy:

- QuadTailsitter defines `Nose / FPV`, `Belly / Down`, and `Chase` through
  `config.ini`; other airframes can add their own views with the same field.
- Nose/FPV uses a `+90 deg` pitch offset so it follows the QuadTailsitter
  fixed-wing nose / virtual pitot axis. In MC hover it points upward; in FW it
  points forward.
- Belly uses a `0 deg` pitch offset so it looks downward in FW.
- Chase uses a rear negative forward offset for inspecting transitions.

## Independent Review Notes

The conservative review decision is to avoid retuning `VT_B_TRANS_DUR`,
`VT_B_DEC_MSS`, rotor cant, ACF geometry, or fixed-wing controller gains in this
slice. qtail27's failure is explained by the RTL altitude schedule and the
measured altitude loss during transition. A broader tune would risk regressing
the accepted qtail26 and qtail27 front-transition/FW tracking behavior.

## Next Test

Use `v3.4.51` with a PX4 parameter reset/distclean. Confirm the ULog shows:

- `RTL_RETURN_ALT=180`
- `RTL_DESCEND_ALT=150`
- `SYS_AUTOSTART=5021`
- `Config Name: QuadTailsitter`

Then run:

1. Take off and climb to at least `180 m AGL`.
2. Manual forward transition.
3. Stabilize in FW, fly one large-radius leg/orbit.
4. Manual RTL from FW mode.
5. Verify auto back-transition does not start near `75 m AGL`.
6. Use `PX4 X-Plane > Camera Views` for nose/belly/chase, or bind
   `px4xplane/camera/view_1..view_3` in X-Plane keyboard settings.

Acceptance:

- No attitude-failure or blind-land failsafe during RTL/back-transition.
- Back-transition starts near the raised RTL descent altitude.
- Nose camera points skyward in MC hover and forward in FW.
- Belly camera points toward the ground in FW.
