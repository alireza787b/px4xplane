# PX4 Airframe Reference Copies

PX4 does not read airframe files from the `px4xplane` plugin folder at runtime.
These files are packaged only as matching reference/install copies for testers.

The source of truth for SITL execution is the PX4 fork branch:

`PX4-Autopilot-Me/ROMFS/px4fmu_common/init.d-posix/airframes/`

When preparing an official PX4 PR, keep the PX4 airframe files in the PX4
repository and avoid adding px4xplane release-kit files to the PX4 branch.
