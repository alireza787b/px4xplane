# Beta / Developer Launcher

The optional `px4xplane` launcher prepares PX4, selects an airframe and starts
SITL on Ubuntu/Debian, including WSL on Windows. It tests the project's latest
airframe settings and supports custom development sources. Normal use follows
the native official PX4 [Quick Start](../README.md#quick-start).
Install X-Plane, its plugin and the matching aircraft before using either path.

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
refreshes the installed launcher pair before updating/building PX4 and reuses
the checkout. You can
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

The launcher checks local edits before updating. It can save ordinary PX4
file edits aside and continue **only after you explicitly agree**. Declining
or closing the prompt preserves them in place and stops the update. Saved
edits are not automatically reapplied to the updated source: the launcher
prints their backup identity for recovery.

Unfinished Git operations, edited submodules and embedded repositories need
individual review; the helper does not pretend a top-level backup covers
them. Ignored build files, including generated dependency repositories, do
not count as local source edits and are left in place. It never force-resets
or runs `git clean`. Selected revisions are checked out detached,
preserving local branch pointers and `origin`. Switching source modes reuses
the same checkout and cache, not a separate clone for every run. The helper
syncs the selected project branch; maintainers integrate upstream changes in
that branch, not by silently merging official PX4 into the user's checkout.

## Setup and Build Recovery

Fresh setup uses PX4's own dependency installer. Existing installations are
checked for missing build tools and Python dependencies; setup asks before
running the official installer when repair is needed. It does not introduce
a separate package installation method. Offline `--no-sync` runs do not
download helpers or run dependency installers.

On a build failure, keep the error report. A clean rebuild, when offered,
cleans only the selected SITL build target after confirmation and retries
once. Saved per-airframe tuning and ULogs remain outside that build target.
The launcher does not run broad `make clean`, `distclean` or submodule clean
commands, which can affect other builds and ignored files.

Use `--verbose` for detailed output. Interactive progress is not a percentage
estimate; logs retain the actual build output.

## Advanced Source Options

`--sync` requests normal synchronization. `--no-sync` skips launcher update
checks and PX4 remote checks/fetches and uses local source. Inspect the reported
commit: no-sync mode does not turn a local custom build into the selected
remote's current revision. Missing dependencies can still prevent an offline
build. For launcher development, `PX4XPLANE_SKIP_LAUNCHER_UPDATE=1` retains the
local helper while allowing PX4 synchronization.
The launcher compares the selected local airframe script against the requested
branch only when a matching remote and cached revision are available. A mismatch
is a warning, not an automatic replacement of intentional local work. This
offline check does not verify the remote's latest state or all inherited defaults.

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
When saved parameters exist, an interactive launch offers **Keep** (Enter),
**Back up and reset** (`r`) or **Quit** (`q`). Changed defaults or an unknown
saved-tuning origin are reported before that choice. A noninteractive launch
preserves tuning unless `--reset-params` is explicit; `--keep-params` skips
the question in an interactive session.
Existing parameter files are backed up before PX4 starts, including Keep
launches; identical snapshots are reused. PX4 still applies its own
autoconfiguration/reset rules. A legacy store outside this source's active
storage path is reported instead of being presented as loaded tuning.

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
