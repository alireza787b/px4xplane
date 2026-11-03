# PX4 Airframe Reference Copies

PX4 does not read airframe files from the `px4xplane` plugin folder at runtime.
These files are packaged only as matching reference/install copies for testers.

The source of truth for SITL execution is the active PX4 checkout:

`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`

When preparing an official PX4 PR, keep the PX4 airframe files in the PX4
repository and avoid adding px4xplane release-kit files to the PX4 branch.

`5021_xplane_qtailsitter` retains the public legacy QuadTailsitter model/default
pairing. The opt-in 211-value compact profile lives separately at
`config/experimental/compact-qtailsitter/5021_xplane_qtailsitter`; it is not the
normal packaged default. See the
[compact developer guide](../../docs/compact-qtailsitter.md) for profile details.

The [guided launcher](../../docs/guided-launcher.md#saved-parameters) preserves
tuning in per-airframe state directories. Updating these reference files does
not automatically replace saved parameters. `--reset-params` explicitly backs
up and resets only the selected runtime.
