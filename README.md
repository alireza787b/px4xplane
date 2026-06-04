# PX4-XPlane

[![Build Status](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml/badge.svg)](https://github.com/alireza787b/px4xplane/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/alireza787b/px4xplane)](https://github.com/alireza787b/px4xplane/releases)
[![License](https://img.shields.io/github/license/alireza787b/px4xplane)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)](https://github.com/alireza787b/px4xplane)

PX4-XPlane connects PX4 SITL to X-Plane. It sends PX4 actuator commands to
writable X-Plane datarefs and returns simulated IMU, GPS, barometer,
magnetometer, airspeed, and ground-truth data.

The current package is `v4.0.0` with Windows, Linux, and macOS builds. It
includes tested examples for Cessna 172, TB2, Ehang 184, Alia 250, and
QuadTailsitter.

## Status

PX4 integration is under review in
[PX4-Autopilot PR #22493](https://github.com/PX4/PX4-Autopilot/pull/22493).
Until that merges, the setup helper temporarily uses the maintained
`alireza787b/PX4-Autopilot-Me` branch `px4xplane-sitl`.

`v4.0.0` includes the px4xplane-side fixes from the recent validation cycle:
sensor timing robustness, low-FPS/pause recovery, stale SITL parameter cleanup,
airframe config validation, camera presets, and the accelerometer-bias/ground
contact fixes that affected earlier test packages.

One PX4-side EKF edge case found during fast VTOL transition testing is tracked
separately in
[PX4-Autopilot PR #27533](https://github.com/PX4/PX4-Autopilot/pull/27533).
That fix is intentionally not bundled into this plugin repository.

## Quick Start

1. Download the package for your OS from
   [Releases](https://github.com/alireza787b/px4xplane/releases).
2. Extract it and copy the `px4xplane` folder to
   `X-Plane/Resources/plugins/`.
3. Start X-Plane and load the matching aircraft.
4. Start PX4 SITL with the helper:

```bash
cd ~
curl -O https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh
bash setup_px4_sitl.sh
```

After setup, run:

```bash
px4xplane
```

The launcher currently supports:

```text
xplane_cessna172
xplane_tb2
xplane_ehang184
xplane_alia250
xplane_qtailsitter
```

## Common Paths

| I want to... | Start here |
| --- | --- |
| Install and fly an included airframe | [Quick Start](#quick-start) and the matching [flight-test card](docs/index.md#flight-test-cards) |
| Run PX4 in WSL2, Docker, or another computer | [Network setup](#network-setup) |
| Create or edit a custom airframe mapping | [Custom airframe guide](docs/custom-airframe-config.md) |
| Use the browser config editor | [Config editor guide](docs/custom-airframe-config.md#open-the-config-editor) |
| Build the plugin from source | [Build guide](docs/BUILD.md) |
| Work on the project or release process | [Developer guide](docs/DEVELOPER.md) |
| See all packaged docs | [Documentation index](docs/index.md) |

## Network Setup

PX4 connects to X-Plane on TCP port `4560`.

Same-machine Linux or macOS usually needs no extra setup. If PX4 runs in WSL2,
Docker, or another computer, set `PX4_SIM_HOSTNAME` in the PX4/SITL shell to
the IP address of the machine running X-Plane:

```bash
export PX4_SIM_HOSTNAME=172.21.144.1
```

Also allow inbound TCP `4560` on the X-Plane host firewall. The setup helper can
auto-detect the Windows host IP for common WSL2 setups, and
`px4xplane --reset-ip` re-prompts if a wrong IP was saved.

## Custom Airframes

The included examples are ready-to-test PX4 targets, not a hard vehicle-class
limit. A new X-Plane model can be integrated when PX4 has a matching
SITL-capable control path and px4xplane can map the needed actuator outputs to
writable X-Plane datarefs.

Use the [custom airframe guide](docs/custom-airframe-config.md) for the config
editor, schema, camera presets, actuator mappings, and validation workflow.

## Videos

New `v4` walkthrough videos will be recorded after the PX4 integration lands in
the official repository. Older videos remain available as a project-history
archive, and future videos will use the same playlist:

[px4xplane video archive and future playlist](https://www.youtube.com/watch?v=eZJpRHFgx6g&list=PLVZvZdBQdm_4RepbwUZaccwH0iQvHtMBh&pp=sAgC)

## Maintenance

```bash
px4xplane --sync         # sync PX4 fork branch, submodules, and saved SITL params
px4xplane --reset-ip     # re-enter the X-Plane host IP
px4xplane --reset-config # clear saved launcher settings
px4xplane --repair       # rerun setup and sync path
```

Before sharing a custom config:

```bash
python3 tools/validate_config.py config/config.ini
```

## License

MIT. See [LICENSE](LICENSE).
