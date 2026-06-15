# PX4-XPlane

[![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)](https://github.com/alireza787b/px4xplane/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)](https://github.com/alireza787b/px4xplane)

PX4-XPlane connects PX4 SITL to X-Plane. It sends PX4 actuator commands to
writable X-Plane datarefs and returns simulated IMU, GPS, barometer,
magnetometer, airspeed, and ground-truth data.

The current package is `v4.0.5` with Windows, Linux, and macOS builds. It
includes tested examples for Cessna 172, TB2, Ehang 184, Alia 250, and
QuadTailsitter.

## Media

[![PX4-XPlane video archive and future v4 playlist](https://img.youtube.com/vi/eZJpRHFgx6g/hqdefault.jpg)](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=sAgC)

New `v4` walkthrough videos are coming soon after the PX4 integration lands.
The current playlist is kept as a project-history archive and will also host
future videos.

## Status

PX4 integration is under review in
[PX4-Autopilot PR #22493](https://github.com/PX4/PX4-Autopilot/pull/22493).
Until that merges, the setup helper temporarily uses the maintained
`alireza787b/PX4-Autopilot-Me` branch `px4xplane-sitl-validation`.

`v4.0.5` includes the px4xplane-side fixes from the recent validation cycle:
sensor timing robustness, low-FPS/pause recovery, stale SITL parameter cleanup,
airframe config validation, camera presets, Alia elevator mapping, Cessna 172
runway steering recovery, and the accelerometer-bias/ground contact fixes that
affected earlier test packages. The launcher also asks whether to apply
temporary PX4 guard PRs while separate PX4 fixes are under review.

One PX4-side EKF edge case found during fast VTOL transition testing is tracked
separately in
[PX4-Autopilot PR #27533](https://github.com/PX4/PX4-Autopilot/pull/27533).
A Standard VTOL front-transition setpoint handoff fix is tracked separately in
[PX4-Autopilot PR #27601](https://github.com/PX4/PX4-Autopilot/pull/27601).
The Tailsitter fixed-wing attitude-frame fix and fixed-wing TECS altitude
reference reset are tracked in
[PX4-Autopilot PR #27669](https://github.com/PX4/PX4-Autopilot/pull/27669) and
[PX4-Autopilot PR #27670](https://github.com/PX4/PX4-Autopilot/pull/27670).
Those fixes are intentionally not bundled into this plugin repository. For
local validation while those PRs are pending, the launcher asks whether to use
the maintained validation branch, then asks whether to stack the pending EKF-GSF
and Standard VTOL guard PRs. The default answer is yes. The validation branch
also includes the accepted Tailsitter attitude-frame guard and the fixed-wing
TECS altitude-frame guard found during final Cessna 172/TB2 testing. Use
`--exact-pr` when you deliberately want the exact X-Plane branch without
temporary PX4 guards. Use `--without-ekf-gsf-guard` or
`--without-vtol-handoff-guard` only to selectively disable those two
cherry-picked guards.

## Quick Start

1. Download the package for your OS from
   [Releases](https://github.com/alireza787b/px4xplane/releases).
2. Extract it and copy the `px4xplane` folder to
   `X-Plane/Resources/plugins/`.
3. Start X-Plane and load the matching aircraft.
4. Start the temporary PX4 SITL helper:

```bash
tmp=$(mktemp) && curl -fsSL -o "$tmp" https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash "$tmp"
```

The helper installs the `px4xplane` launcher, syncs the PX4 SITL validation branch,
and opens the airframe menu. If you accept the command install prompt, future
sessions can be started by typing `px4xplane` from any terminal.

This helper is temporary while
[PX4-Autopilot PR #22493](https://github.com/PX4/PX4-Autopilot/pull/22493) is
under review; after the PR merges, this section will switch to the official PX4
SITL commands.

For WSL2, Docker, remote machines, or firewall/IP setup, use the
[network setup guide](docs/index.md#network-and-platform-notes). If a wrong host
IP was saved, rerun the launcher with `px4xplane --reset-ip`.
For common setup or connection errors, see
[Troubleshooting](docs/index.md#troubleshooting).

For final validation while the separate PX4 guard PRs are still under review,
use the validation shortcut. It selects `px4xplane-sitl-validation`, applies
the EKF-GSF and Standard VTOL guard PRs, and uses the Tailsitter and TECS
validation fixes already included in that branch:

```bash
px4xplane --validation --reset-config
```

The launcher keeps the selected PX4 checkout aligned with the validation branch
and stacks the pending PX4 guard commits locally. It does not change the
official X-Plane PR scope. For reviewer-scope checks of the exact X-Plane PR
without temporary guard PRs, run `px4xplane --exact-pr --reset-config`.

If you already have a PX4 checkout and do not want a second clone, pass it to
the helper:

```bash
px4xplane --px4-path ~/PX4-Autopilot --sync --reset-config
```

The helper uses a separate `px4xplane` git remote inside that checkout and
leaves your `origin` remote unchanged. To put the selected checkout back on
official PX4 `master`, run:

```bash
px4xplane --px4-path ~/PX4-Autopilot --restore-official
```

## Common Paths

| I want to... | Start here |
| --- | --- |
| Install and fly an included airframe | [Quick Start](#quick-start) and the matching [flight-test card](docs/index.md#flight-test-cards) |
| Run PX4 in WSL2, Docker, or another computer | [Network setup](docs/index.md#network-and-platform-notes) |
| Fix setup or connection errors | [Troubleshooting](docs/index.md#troubleshooting) |
| Create or edit a custom airframe mapping | [Custom airframe guide](docs/custom-airframe-config.md) |
| Use the browser config editor | [Config editor guide](docs/custom-airframe-config.md#open-the-config-editor) |
| Build the plugin from source | [Build guide](docs/BUILD.md) |
| Work on the project or release process | [Developer guide](docs/DEVELOPER.md) |
| See all packaged docs | [Documentation index](docs/index.md) |

## Custom Airframes

The included examples are ready-to-test PX4 targets, not a hard vehicle-class
limit. A new X-Plane model can be integrated when PX4 has a matching
SITL-capable control path and px4xplane can map the needed actuator outputs to
writable X-Plane datarefs.

Use the [custom airframe guide](docs/custom-airframe-config.md) for the config
editor, schema, camera presets, actuator mappings, and validation workflow.

## Maintenance

The launcher includes maintenance flags for syncing the PX4 fork branch,
resetting saved IP/config choices, and repairing the setup. See the
[documentation index](docs/index.md) for the operator and developer workflows.

## License

MIT. See [LICENSE](LICENSE).
