# Guided PX4 Launcher

The `px4xplane` launcher prepares PX4, selects an airframe and starts SITL on
Ubuntu/Debian, including WSL on Windows. Start with the
[Quick Start](../README.md#quick-start) to install X-Plane, the plugin and the
matching aircraft. Other platforms can use the
[advanced manual setup](../README.md#advanced-manual-setup).

## Install and Run

Run the setup helper in your Ubuntu/Debian or WSL shell:

```bash
(base=https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup; tmp=$(mktemp) && trap 'rm -f "$tmp"' EXIT && curl -fsSL "$base/setup_px4_sitl.sh" -o "$tmp" && PX4XPLANE_LAUNCHER_BASE_URL="$base" bash "$tmp")
```

Accept command installation, then use this for later sessions:

```bash
px4xplane
```

The launcher checks dependencies, reuses a PX4 checkout and build cache, syncs
the selected source, detects networking and offers the airframe menu. Load the
matching aircraft in X-Plane, select its plugin mapping, and choose
`Connect to SITL`. Reconnect if the first build outlasts the plugin's wait window.
After a run, use `shutdown` in the PX4 console.

To update an existing installation, run the same setup command again. It
refreshes the installed launcher pair and reuses the PX4 checkout. You can
also run `px4xplane --update-launcher` to update just the launcher, then start
`px4xplane` normally.

## Source Modes

| Mode | PX4 source | Airframe defaults |
| --- | --- | --- |
| `--maintained` (default) | Project fork, branch `px4xplane-airframes` | Current project airframe files on an upstream PX4 base |
| `--official` | Bare `PX4/PX4-Autopilot` `main` | Only the defaults present upstream at the selected revision |

The maintained repository is
`https://github.com/alireza787b/PX4-Autopilot-Me.git`. Normal operation needs no
PR stack or additional profile runner. Maintained airframe updates are not a
claim that the same values have merged upstream. `--official` does not apply
project airframe files or pending patches; it is an explicit upstream comparison.

The launcher reports the selected repository, revision and airframe file.
Check them when switching modes. If bare upstream lacks the requested target,
use an available target or return to maintained mode; do not silently copy in
different defaults. See [developer validation](DEVELOPER.md#developer-validation)
for comparison and verification requirements.

## Airframe Selection

Without `--airframe`, choose from the menu. To select a target directly:

```bash
px4xplane --airframe xplane_qtailsitter
```

List the canonical names with `px4xplane --list-airframes`:

| Aircraft | `--airframe` value | `SYS_AUTOSTART` | Plugin section |
| --- | --- | --- | --- |
| Cessna 172 | `xplane_cessna172` | 5001 | `Cessna172` |
| TB2 | `xplane_tb2` | 5002 | `TB2` |
| Ehang 184 | `xplane_ehang184` | 5010 | `ehang184` |
| Alia 250 | `xplane_alia250` | 5020 | `Alia250` |
| QuadTailsitter | `xplane_qtailsitter` | 5021 | `QuadTailsitter` |

Select the same aircraft in X-Plane and the plugin. A target name or ID is not
proof of model geometry or saved tuning. QuadTailsitter has one canonical
profile; follow its [test card](QUADTAILSITTER_XPLANE12_TEST.md).

## Existing Checkouts and Sync

Select an existing checkout explicitly when needed:

```bash
px4xplane --px4-path ~/PX4-Autopilot --airframe xplane_alia250
```

The launcher checks tracked, untracked and submodule changes before updating.
It stops for work that prevents a safe update; it does not force-stash,
force-reset or run `git clean`. Selected revisions are checked out detached,
preserving local branch pointers and `origin`. Switching source modes reuses
the same checkout and cache, not a separate clone for every run.

`--sync` requests normal synchronization. `--no-sync` skips launcher update
checks and PX4 remote checks/fetches and uses local source. Inspect the reported
commit: no-sync mode does not turn a local custom build into the selected
remote's current revision. Missing dependencies can still prevent an offline
build. For launcher development, `PX4XPLANE_SKIP_LAUNCHER_UPDATE=1` retains the
local helper while allowing PX4 synchronization.

Custom source selection is available through `--px4-repo URL`,
`--px4-branch NAME` and `--px4-path PATH`; it is not part of normal setup.
Do not select a checkout reserved for a pinned comparison unless you intend
to change its checked-out source. Preserving branch pointers does not keep
the worktree on its previous revision.

## Saved Parameters

State is grouped by checkout/source and airframe under
`$XDG_STATE_HOME/px4xplane/sitl` (default `~/.local/state/px4xplane/sitl`).
The launcher prints the selected runtime path and uses it as PX4's `-w`
working directory. Saved tuning remains between launches. Syncing source or
detecting changed airframe defaults does not automatically reset it.

To back up and reload defaults for one selected airframe:

```bash
px4xplane --airframe xplane_qtailsitter --reset-params
```

Review the model/profile pairing first. Keep the printed backup to recover
tuning. Only the selected runtime's parameter store is reset; other airframes
are untouched. Do not delete shared PX4 parameters as routine setup.

PX4 executes `ROMFS/px4fmu_common/init.d-posix/airframes/<ID>_<target>` from
the selected PX4 build. The plugin's `px4_airframes` and repository
`config/px4_params` folders are reference copies, not scripts the plugin runs.
`param set-default` supplies defaults; existing saved values can override them.

`--reset-ip` forgets the X-Plane host; `--reset-config` resets launcher choices.
Neither resets PX4 parameters. `--repair` reruns dependency/setup checks and
does not discard local work. Uninstall preserves checkouts, saved tuning and logs.

## X-Plane and QGC Networking

| Connection | Destination | Default port |
| --- | --- | --- |
| PX4 simulator link | X-Plane host | TCP `4560` |
| PX4 MAVLink telemetry | QGC host | UDP `14550` |

The launcher detects localhost/mirrored WSL2 and the Windows host address for
WSL2 NAT, refreshing automatic choices when networking changes. Manual choices
remain saved until reset. QGC defaults to the selected X-Plane host; use
`--qgc-host <qgc-host-ip>` if it runs elsewhere.

Allow inbound TCP `4560` on the X-Plane host and UDP `14550` on the QGC host.
Direct MAVLink is the default. An explicit QGC destination uses a runtime-local
copy of PX4's MAVLink startup script, not a source patch. Confirm that QGC
actually shows a connected vehicle; an address or working simulator link
alone does not establish telemetry reception.

`--with-mavlink-router` optionally uses an already-installed `mavlink-routerd`
for remote QGC, not localhost. No router installer runs as part of setup.
Truth capture is optional diagnostic recording, not a launch prerequisite.

See [Troubleshooting](index.md#troubleshooting) for connection, checkout and
saved-parameter issues.
