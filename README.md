# PX4-XPlane

<p align="center">
  <img src="docs/assets/px4xplane-logo.svg" alt="PX4-XPlane logo" width="360">
</p>

[![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)](https://github.com/alireza787b/px4xplane/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)](https://github.com/alireza787b/px4xplane)

PX4-XPlane connects PX4 SITL to X-Plane. It sends PX4 actuator commands to
writable X-Plane datarefs and returns simulated IMU, GPS, barometer,
magnetometer, airspeed, and ground-truth data.

The current package is `v4.1.0` with Windows, Linux, and macOS builds. It
includes tested examples for Cessna 172, TB2, Ehang 184, Alia 250, and
QuadTailsitter.

## Latest Video

[![PX4-XPlane latest video and project playlist](https://img.youtube.com/vi/eZJpRHFgx6g/hqdefault.jpg)](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=sAgC)

New `v4` walkthrough videos are coming soon. The current thumbnail links to the
project video archive and playlist; future videos will be added there.

## Quick Start

1. Install X-Plane 11 or 12.
   Use the official [X-Plane download](https://www.x-plane.com/desktop/try-it/)
   and [installation guide](https://www.x-plane.com/kb/digital-download-install/).

2. Install or update PX4 SITL.
   Follow the official
   [PX4 development environment](https://docs.px4.io/main/en/dev_setup/dev_env.html)
   setup for your OS, WSL2, container, or remote Linux machine.

3. Download the latest px4xplane package for your OS from
   [Releases](https://github.com/alireza787b/px4xplane/releases).

4. Extract the archive and copy the complete `px4xplane` folder into the
   `Resources/plugins` folder of your X-Plane installation, for example
   `X-Plane 12/Resources/plugins/px4xplane`.

5. Copy any packaged X-Plane aircraft folders you want to test into an X-Plane
   aircraft directory, then start X-Plane and load the matching aircraft. In
   X-Plane, choose `Plugins > PX4 X-Plane > Airframes > ...` and select the
   matching airframe, then click `Plugins > PX4 X-Plane > Connect to SITL`.

6. Start PX4 SITL from your official PX4 checkout:

```bash
cd ~/PX4-Autopilot
make px4_sitl_default xplane_alia250
```

Starting PX4 first can also work, but starting `Connect to SITL` in X-Plane
first gives visible connection status. The first PX4 build can take longer than
the plugin's 60-second wait window; if the plugin times out, click
`Connect to SITL` again after PX4 is running.

Available PX4 targets include `xplane_alia250`, `xplane_cessna172`,
`xplane_tb2`, `xplane_ehang184`, and `xplane_qtailsitter`. See the official
[PX4 X-Plane simulation](https://docs.px4.io/main/en/sim_xplane/) page for the
current target list, running notes, and the custom-airframe integration path.

## Network Notes

PX4 connects to the px4xplane plugin on TCP port `4560`.

If PX4 SITL and X-Plane run on the same Linux or macOS machine, the default
localhost setup is normally enough.

If PX4 runs in WSL2, Docker, or another computer, set `PX4_SIM_HOSTNAME` in the
PX4/SITL shell to the IP address of the machine running X-Plane before starting
PX4:

```bash
export PX4_SIM_HOSTNAME=<xplane-host-ip>
make px4_sitl_default xplane_alia250
```

For common WSL2 setups where X-Plane runs on Windows and PX4 runs inside WSL:

```bash
export PX4_SIM_HOSTNAME=$(ip route | awk '/default/ {print $3; exit}')
```

Run that `export` before the normal PX4 `make px4_sitl_default xplane_*`
command in the same terminal session.

Also allow inbound TCP `4560` on the X-Plane host firewall. More detail is in
the [network setup guide](docs/index.md#network-and-platform-notes).

## px4xplane CLI and Experimental Validation

The official PX4 commands above are the normal path. The optional `px4xplane`
launcher is useful when you want a guided WSL/IP prompt, automatic PX4 sync,
airframe selection, stale SITL parameter cleanup, or local validation of
pending PX4 fixes:

```bash
tmp=$(mktemp) && curl -fsSL -o "$tmp" https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash "$tmp"
```

The helper installs the `px4xplane` launcher, syncs the selected PX4 checkout,
asks about pending PX4 validation PRs, and opens the airframe menu. If you
accept the command install prompt, future sessions can be started by typing
`px4xplane` from any terminal.

For full local validation while separate PX4 fixes are still pending, use:

```bash
px4xplane --validation --reset-config
```

That command uses official PX4 `main` as the base and stacks the pending PX4 PRs
locally for the run. For a clean official-PX4 baseline without pending PRs, use
`px4xplane --official --reset-config`.

If you already have a PX4 checkout and do not want a second clone, pass it to
the helper:

```bash
px4xplane --px4-path ~/PX4-Autopilot --sync --reset-config
```

The helper uses a separate `px4xplane` git remote inside that checkout and
leaves your `origin` remote unchanged. To put the selected checkout back on
official PX4 `main`, run:

```bash
px4xplane --px4-path ~/PX4-Autopilot --restore-official
```

If a wrong host IP was saved, rerun with `px4xplane --reset-ip`. For common
setup or connection errors, see [Troubleshooting](docs/index.md#troubleshooting).

## Status

PX4 X-Plane SITL support is merged in
[PX4-Autopilot PR #22493](https://github.com/PX4/PX4-Autopilot/pull/22493).
Use official `PX4/PX4-Autopilot` `main` for normal SITL runs.

`v4.1.0` includes the px4xplane-side fixes from the recent validation cycle:
sensor timing robustness, low-FPS/pause recovery, stale SITL parameter cleanup,
airframe config validation, camera presets, Alia elevator mapping, Cessna 172
runway steering recovery, post-merge official-PX4 launcher/docs cleanup, and
the accelerometer-bias/ground contact fixes that affected earlier test packages.

Several PX4-side fixes found during final X-Plane validation are tracked in
separate PX4 PRs:

- [PX4-Autopilot PR #27533](https://github.com/PX4/PX4-Autopilot/pull/27533):
  EKF-GSF emergency yaw-reset guard for fast VTOL transitions.
- [PX4-Autopilot PR #27601](https://github.com/PX4/PX4-Autopilot/pull/27601):
  Standard VTOL front-transition setpoint handoff.
- [PX4-Autopilot PR #27670](https://github.com/PX4/PX4-Autopilot/pull/27670):
  fixed-wing TECS reset on altitude-reference changes.
- [PX4-Autopilot PR #27793](https://github.com/PX4/PX4-Autopilot/pull/27793):
  active upstream tailsitter attitude-conversion fix.

Those PX4 changes are intentionally not bundled into this plugin repository.
The launcher can stack them locally for validation while they are pending.

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

The launcher includes maintenance flags for syncing the selected PX4 checkout,
resetting saved IP/config choices, and repairing the setup. See the
[documentation index](docs/index.md) for the operator and developer workflows.

## License

MIT. See [LICENSE](LICENSE).
