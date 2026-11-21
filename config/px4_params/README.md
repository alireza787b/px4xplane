# PX4 Airframe Reference Copies

These are the five canonical project airframe reference files, also packaged
under the plugin's `px4_airframes` directory. Normal setup uses the maintained
PX4 fork, branch `px4xplane-airframes`, which carries the matching executable
airframe files. See the [launcher guide](../../docs/guided-launcher.md).

## What Runs

The executable source is the selected PX4 checkout's:

`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`

Rebuilding PX4 embeds those scripts in its ROMFS. PX4 selects the script using
`SYS_AUTOSTART` when it starts and loads inherited vehicle defaults plus its
airframe defaults. Saved parameter overrides can take precedence over
`param set-default` values. The plugin does not execute, import or apply the
copies in this directory; replacing a plugin reference file alone does not
change a PX4 run.

| Aircraft | Reference file | Target / ID |
| --- | --- | --- |
| Cessna 172 | `5001_xplane_cessna172` | `xplane_cessna172` / 5001 |
| TB2 | `5002_xplane_tb2` | `xplane_tb2` / 5002 |
| Ehang 184 | `5010_xplane_ehang184` | `xplane_ehang184` / 5010 |
| Alia 250 | `5020_xplane_alia250` | `xplane_alia250` / 5020 |
| QuadTailsitter | `5021_xplane_qtailsitter` | `xplane_qtailsitter` / 5021 |

QuadTailsitter has one canonical 209-value profile for its matching 2 kg,
0.6 m-span aircraft. There is no alternate profile to opt into. Keep the
existing target, ID and `[QuadTailsitter]` plugin section. See the
[QuadTailsitter card](../../docs/QUADTAILSITTER_XPLANE12_TEST.md).

## Source Selection and Saved Tuning

`--maintained` is the launcher's default and supplies the project airframe
files. `--official` selects bare upstream PX4 without overlays. Upstream may
have different defaults; do not assume these reference updates are merged
there. Record the actual source revision and compare its ROMFS file when
reviewing a run.

The [guided launcher](../../docs/guided-launcher.md#saved-parameters) preserves
tuning in per-airframe state directories. Updating these reference files does
not automatically replace saved parameters. `--reset-params` explicitly backs
up and resets only the selected runtime.

For source maintenance and current parameter compatibility, see the
[airframe reference contract](../../docs/DEVELOPER.md#airframe-reference-contract).
When preparing an upstream PX4 PR, keep executable airframe changes in PX4;
do not add plugin release-kit files or aircraft assets to that branch.
