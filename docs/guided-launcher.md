# Guided PX4 Launcher

The optional `px4xplane` launcher automates PX4 setup and airframe selection.
It targets Ubuntu/Debian, including WSL on Windows. On macOS, use the manual
official PX4 workflow below.
For the manual workflow, follow the official
[PX4 development setup](https://docs.px4.io/main/en/dev_setup/dev_env.html) and
[X-Plane simulation guide](https://docs.px4.io/main/en/sim_xplane/) first.

## Install and Run

Install X-Plane and the plugin using the [Quick Start](../README.md#quick-start),
then run the setup helper in your PX4/SITL shell:

```bash
tmp=$(mktemp) && curl -fsSL -o "$tmp" https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh && bash "$tmp"
```

Accept the command installation prompt to use `px4xplane` in later sessions:

```bash
px4xplane
```

Plain `px4xplane` prepares official `PX4/PX4-Autopilot` `main` automatically,
without pending PRs. It reuses an existing checkout, checks dependencies,
syncs the selected source and offers an airframe menu. Select the matching
aircraft in X-Plane and the plugin, then choose `Connect to SITL`.
If the first build outlasts the plugin's connection timeout, reconnect once
PX4 is running.

## Existing Checkouts and Sync

To select an existing checkout explicitly:

```bash
px4xplane --px4-path ~/PX4-Autopilot
```

The launcher validates the checkout and submodules before updating. It does
not force-stash changes, force-reset branches or run `git clean`. If local
changes prevent a safe update, it stops for you to resolve them or choose a
different checkout. The requested revision is checked out detached, leaving
local branch pointers and your `origin` remote unchanged. A later `git switch`
can return to your development branch.

`--sync` explicitly requests the normal synchronization path. `--no-sync`
skips launcher update checks and PX4 remote checks/fetches, using the available
local source instead. It does not guarantee an offline build: missing build
dependencies may still need installation. It cannot fetch/apply the TECS
validation proposal. For a feature-branch helper, set
`PX4XPLANE_SKIP_LAUNCHER_UPDATE=1` to retain that launcher version while
allowing PX4 synchronization.

Use `--px4-repo URL`, `--px4-branch NAME` and `--px4-path PATH` for an
intentional custom source selection. These overrides are not required for
normal operation. `--restore-official` returns the selected checkout to
official PX4 `main`, subject to the same checkout-safety checks; it is not a
way to discard local work.

## Saved Parameters

Each airframe has its own state directory under
`$XDG_STATE_HOME/px4xplane/sitl` (default `~/.local/state/px4xplane/sitl`), grouped
by checkout/source and target. The launcher prints the selected runtime path.
It builds using official PX4 `make`, then runs the stock PX4 binary with that
runtime as its `-w` working directory. Saved parameters are isolated between
airframes and retained between launches. Syncing source or noticing changed
airframe defaults does not automatically delete tuning.

To intentionally reload defaults for the airframe you select:

```bash
px4xplane --reset-params
```

The reset backs up only the selected runtime's saved parameter store before
resetting it. Keep that backup if you need to recover tuning. Other airframes'
state is untouched. Do not delete shared PX4 parameter files as routine setup.

`--reset-ip` clears the remembered X-Plane host; `--reset-config` clears launcher
configuration and prompts again. Neither is a parameter-reset request.
`--repair` reruns dependency/setup checks, not a destructive checkout repair.

## X-Plane and QGC Networking

These are separate connections:

| Connection | Destination | Default port |
| --- | --- | --- |
| PX4 simulator link | X-Plane host | TCP `4560` |
| PX4 MAVLink telemetry | QGC host | UDP `14550` |

For manual PX4 commands, set `PX4_SIM_HOSTNAME` to the X-Plane host before
starting SITL. The launcher detects localhost/mirrored WSL2 and Windows-host
addresses for WSL2 NAT, refreshing automatic choices as networking changes.
Manual host choices remain saved until reset.

The QGC target defaults to the selected X-Plane host. If QGC runs elsewhere,
supply its destination explicitly:

```bash
px4xplane --qgc-host <qgc-host-ip>
```

Allow inbound TCP `4560` on the X-Plane host and UDP `14550` on the QGC host.
Direct MAVLink is the default. For an explicit QGC target, the launcher uses a
runtime-local copy of the stock MAVLink startup script; it does not require a
PX4 source patch. Configuring or printing an address does not prove receipt:
confirm that QGC displays a connected vehicle. A working simulator TCP link
alone does not establish QGC telemetry.

MAVLink Router is optional. Only `--with-mavlink-router` opts in, using an
already-installed `mavlink-routerd` binary for a remote QGC address (not
localhost, which would create a loop/port conflict). The launcher does not download a
router installer or make it a normal setup dependency.

## Developer Validation

```bash
px4xplane --validation
```

This explicitly selects only the open
[TECS altitude-reference proposal #27670](https://github.com/PX4/PX4-Autopilot/pull/27670)
on official PX4 `main`. `--with-tecs-alt-guard` is the specific guard option.
It does not apply closed GSF #27533 or already-merged VTOL/tailsitter proposals.
Retired positive guard options stop with an explanation. `--no-sync` cannot
be used to fetch/apply this proposal.

Use `--official` for the official baseline without pending patches, or
`--restore-official` to return a previously selected validation checkout to
official `main`. Inspect checkout state before comparing builds; parameter
reset remains an independent, explicit choice.

No private service, Telegram integration, upload, benchmark or truth-capture
tool is needed for the public launcher. See the [Developer Guide](DEVELOPER.md)
for development policy and the [documentation index](index.md#troubleshooting)
for setup troubleshooting.
