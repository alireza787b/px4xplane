# PX4-XPlane

<p align="center">
  <img src="docs/assets/px4xplane-logo.svg" alt="PX4-XPlane logo" width="420">
</p>

[![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)](https://github.com/alireza787b/px4xplane/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)](https://github.com/alireza787b/px4xplane)

PX4-XPlane connects PX4 SITL to X-Plane. It sends PX4 actuator commands to
writable X-Plane datarefs and returns simulated IMU, GPS, barometer,
magnetometer, airspeed, and ground-truth data.

The current package is `v4.2.1` with Windows, Linux, and macOS builds. The five
airframe mappings are Cessna 172, TB2, Ehang 184, Alia 250 and QuadTailsitter.

## Latest Video

[![PX4-XPlane latest video and project playlist](https://img.youtube.com/vi/eZJpRHFgx6g/hqdefault.jpg)](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=sAgC)

New `v4` walkthrough videos are coming soon. The current thumbnail links to the
project video archive and playlist; future videos will be added there.

## Quick Start

1. Install X-Plane 11 or 12.
   Use the official [X-Plane download](https://www.x-plane.com/desktop/try-it/)
   and [installation guide](https://www.x-plane.com/kb/digital-download-install/).

2. Download the plugin for your OS from
   [Releases](https://github.com/alireza787b/px4xplane/releases). Close X-Plane,
   back up custom configs, and replace the complete `px4xplane` plugin folder
   under `Resources/plugins`. Do not merge old and new package files.
   For changes still marked [Unreleased](CHANGELOG.md#unreleased), install a
   [source build](docs/BUILD.md) instead; an older release ZIP does not contain
   the updated configuration.

3. Install the matching X-Plane aircraft and open it. Select its mapping under
   `Plugins > PX4 X-Plane > Airframes`, then choose `Connect to SITL`.
   See the [airframe cards](docs/index.md#flight-test-cards) for model details.

4. Open Ubuntu/Debian or WSL and run the setup helper:

   ```bash
   (base=https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup; tmp=$(mktemp) && trap 'rm -f "$tmp"' EXIT && curl -fsSL "$base/setup_px4_sitl.sh" -o "$tmp" && PX4XPLANE_LAUNCHER_BASE_URL="$base" bash "$tmp")
   ```

5. Accept command installation and select the same airframe in the launcher.
   It prepares PX4, detects the X-Plane host, builds using the existing cache,
   and opens the PX4 console. Confirm QGroundControl shows a connected vehicle.
   If the first build exceeds the plugin's wait window, choose
   `Connect to SITL` again once PX4 is running.

For later sessions, run `px4xplane`. The default `--maintained` mode uses the
project's [maintained PX4 branch](https://github.com/alireza787b/PX4-Autopilot-Me/tree/px4xplane-airframes),
which carries the current project airframe defaults on an upstream PX4 base.
The launcher reuses a checkout and build cache, preserves local work, and keeps
saved parameters separate for each airframe. See the
[launcher guide](docs/guided-launcher.md) for `--airframe`, source selection,
networking and backed-up resets.

## Airframes

| Aircraft | PX4 target | ID |
| --- | --- | --- |
| Cessna 172 | `xplane_cessna172` | 5001 |
| TB2 | `xplane_tb2` | 5002 |
| Ehang 184 | `xplane_ehang184` | 5010 |
| Alia 250 | `xplane_alia250` | 5020 |
| QuadTailsitter | `xplane_qtailsitter` | 5021 |

QuadTailsitter uses one canonical profile for its matching 2 kg, 0.6 m-span,
four-motor aircraft. Use the [QuadTailsitter card](docs/QUADTAILSITTER_XPLANE12_TEST.md)
for model selection and checks. PX4 loads defaults from the selected PX4
checkout, not from the plugin's reference folder.

## Networking

The simulator connection uses TCP `4560` on the X-Plane host. QGroundControl
(QGC) receives MAVLink separately on UDP `14550`. The launcher detects WSL
networking; use `--qgc-host <qgc-host-ip>` when QGC runs on another machine.
Allow these ports through the corresponding host firewalls and check QGC's
vehicle connection. See [network setup](docs/index.md#network-and-platform-notes)
and [troubleshooting](docs/index.md#troubleshooting).

## Advanced Manual Setup

For macOS, containers or a manually managed development environment, follow
the official [PX4 development setup](https://docs.px4.io/main/en/dev_setup/dev_env.html)
and [X-Plane simulation guide](https://docs.px4.io/main/en/sim_xplane/).
Choose the source deliberately: the maintained branch supplies this project's
current airframe defaults; `--official` selects bare upstream PX4 without them.
See [source modes](docs/guided-launcher.md#source-modes) before comparing runs.

## Common Paths

| I want to... | Start here |
| --- | --- |
| Install and run an airframe | [Quick Start](#quick-start) and the matching [flight-test card](docs/index.md#flight-test-cards) |
| Select a checkout, source or airframe | [Guided launcher guide](docs/guided-launcher.md) |
| Run PX4 in WSL2, Docker, or another computer | [Network setup](docs/index.md#network-and-platform-notes) |
| Fix setup or connection errors | [Troubleshooting](docs/index.md#troubleshooting) |
| Create or edit a custom airframe mapping | [Custom airframe guide](docs/custom-airframe-config.md) |
| Use the browser config editor | [Config editor guide](docs/custom-airframe-config.md#open-the-config-editor) |
| Build the plugin from source | [Build guide](docs/BUILD.md) |
| Work on the project or release process | [Developer guide](docs/DEVELOPER.md) |
| See all packaged docs | [Documentation index](docs/index.md) |

## Custom Airframes

The five example mappings are not a vehicle-class limit. A new X-Plane model
can be integrated when PX4 has a matching
SITL-capable control path and px4xplane can map the needed actuator outputs to
writable X-Plane datarefs.

Use the [custom airframe guide](docs/custom-airframe-config.md) for the config
editor, schema, camera presets, actuator mappings, and validation workflow.

The [configuration editor](docs/custom-airframe-config.md#open-the-config-editor)
edits the plugin's `64/config.ini` actuator and sensor mapping. PX4 flight
parameters are separate.

## License

MIT. See [LICENSE](LICENSE).
