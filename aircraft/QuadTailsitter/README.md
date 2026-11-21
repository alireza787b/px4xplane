# QuadTailsitter X-Plane Aircraft

Use the [QuadTailsitter test card](../../docs/QUADTAILSITTER_XPLANE12_TEST.md)
for the single canonical model/profile pairing and beta launcher workflow.
This directory contains documentation only, not a bundled ACF or aircraft
installer. Obtain the matching aircraft separately from its owner.

| Item | Configuration |
| --- | --- |
| Aircraft | 2 kg, 0.6 m-span tailsitter |
| Propulsion | Four fixed 10x8 propellers, 6S power |
| Control | Differential thrust, no control surfaces |
| PX4 target / ID | `xplane_qtailsitter` / `5021` |
| Plugin section | `[QuadTailsitter]` |

The maintained PX4 branch supplies the canonical defaults. The plugin's
reference copy does not configure PX4 at runtime. See the
[reference notes](../../config/px4_params/README.md) before changing saved tuning.

## Camera Views

px4xplane provides config-driven camera views under
`PX4 X-Plane > Camera Views`. The views are not hardcoded to this aircraft;
they are read from the active airframe's `cameraViews` line in `config.ini`.

The QuadTailsitter mapping defines:

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

Aircraft icons, panels and X-Plane quick-look preferences belong to the
separately installed model. They are not provided by this documentation folder.
