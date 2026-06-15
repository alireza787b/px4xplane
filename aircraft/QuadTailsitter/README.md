# QuadTailsitter X-Plane Aircraft

This aircraft is paired with PX4 airframe `5021_xplane_qtailsitter`.

## Camera Views

px4xplane v4.0.6 provides config-driven camera views under
`PX4 X-Plane > Camera Views`. The views are not hardcoded to this aircraft;
they are read from the active airframe's `cameraViews` line in `config.ini`.

QuadTailsitter ships:

- `Nose / FPV`: in hover it points upward; in fixed-wing it points forward along
  the virtual pitot axis.
- `Belly / Down`: in fixed-wing it looks down toward the ground.
- `Chase`: rear engineering chase view.

Bindable commands are generic: `px4xplane/camera/view_1`,
`px4xplane/camera/view_2`, `px4xplane/camera/view_3`, and
`px4xplane/camera/release`. Bind them in X-Plane's keyboard settings if the
default quick-look keys are inconvenient or unavailable.

The config format is:

`Label|forward_m|right_m|up_m|pitch_offset_deg|heading_offset_deg|roll_offset_deg|zoom`

## Quick-Look Views

`QuadTailsitter_prefs.txt` ships three X-Plane quick-look presets:

- location `#1`: nose/FPV camera, aligned with the fixed-wing nose axis
- location `#2`: belly/down-looking camera, pitched down relative to the
  fixed-wing body
- location `#3`: rear engineering chase view

Use X-Plane's quick-look commands to bind these to the keys you prefer. On many
installations the default commands are `sim/view/quick_look_0`,
`sim/view/quick_look_1`, and `sim/view/quick_look_2`.

## Aircraft Selection Icons

`QuadTailsitter_icon11.png` and `QuadTailsitter_icon11_thumb.png` are included so
the aircraft does not show a generic question-mark tile in X-Plane's aircraft
selection UI. For a photoreal exact icon from your installed simulator, load the
aircraft in X-Plane and run `Developer > Regenerate icons for current aircraft`.

## Panel Scope

The current 2-D panel is intentionally minimal and uses X-Plane's native panel
records. A polished eVTOL operator panel with RPM/thrust/battery tiles, map, and
camera MFDs should be authored in Plane Maker as a dedicated visual-cockpit
slice, then committed after the ACF is saved by Plane Maker.
