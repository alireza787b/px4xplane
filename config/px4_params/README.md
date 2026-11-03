# PX4 Airframe Reference Copies

PX4 does not read airframe files from the `px4xplane` plugin folder at runtime.
These files are packaged only as matching reference/install copies for testers.

The source of truth for SITL execution is the active PX4 checkout:

`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`

When preparing an official PX4 PR, keep the PX4 airframe files in the PX4
repository and avoid adding px4xplane release-kit files to the PX4 branch.

Draft exception for pre-merge validation: the compact QuadTailsitter runner
explicitly reads its matching reference into an isolated parameter store.
The replacement `5021_xplane_qtailsitter` reference targets the privately
supplied compact QuadTailsitter model, not the old public 5 kg model. See
`docs/compact-qtailsitter.md` in the plugin package/repository. A stock PX4
checkout will not read these new defaults until its companion PR is merged.
