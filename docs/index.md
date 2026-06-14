# px4xplane Documentation

This is the stable documentation entry point for the PX4-XPlane plugin, packaged aircraft, setup helper, and configuration tools.

## Start Here

- [README](../README.md)
- [Build Guide](BUILD.md)
- [Developer Guide](DEVELOPER.md)
- [Custom Airframe Config](custom-airframe-config.md)
- [Config Editor](config-editor.html)
- [Config Schema](developer/config-schema.md)

## Flight-Test Cards

Use these cards when validating a package or tuning a PX4 airframe:

- [Alia 250 X-Plane 12 Test Card](ALIA_XPLANE12_TEST.md)
- [Ehang 184 X-Plane 12 Test Card](EHANG184_XPLANE12_TEST.md)
- [QuadTailsitter X-Plane 12 Test Card](QUADTAILSITTER_XPLANE12_TEST.md)
- [Cessna 172 X-Plane 12 Test Card](CESSNA172_XPLANE12_TEST.md)
- [TB2 X-Plane 12 Test Card](TB2_XPLANE12_TEST.md)

## Network and Platform Notes

PX4 connects to X-Plane on TCP port `4560`.

Same-machine Linux or macOS usually needs no extra network setup. If PX4 runs
inside WSL2, Docker, or another computer, set `PX4_SIM_HOSTNAME` in the PX4/SITL
environment to the IP address of the machine running X-Plane:

```bash
export PX4_SIM_HOSTNAME=<xplane-host-ip>
```

Also allow inbound TCP `4560` on the X-Plane host firewall. The temporary
`px4xplane` launcher can auto-detect common WSL2 host IPs, and
`px4xplane --reset-ip` clears a saved wrong IP and asks again.

## Troubleshooting

- Wrong X-Plane host IP: run `px4xplane --reset-ip` and choose the correct host
  IP again.
- Stale local PX4 checkout, submodules, or SITL parameters: run `px4xplane`.
  The launcher syncs the selected PX4 branch by default during the temporary
  validation phase. Use `px4xplane --sync --reset-config` when you also want to
  clear remembered IP/config choices.
- Broken setup or missing launcher command: rerun the setup helper, or use
  `px4xplane --repair` if the command already exists.
- Plugin does not appear in X-Plane: confirm the whole `px4xplane` folder is in
  `X-Plane/Resources/plugins/`, then check X-Plane `Log.txt` for plugin load
  errors.
- Custom mapping behaves unexpectedly: validate `px4xplane/64/config.ini` from
  the plugin menu or with `python3 tools/validate_config.py config/config.ini`.
- Build problems: use the [Build Guide troubleshooting section](BUILD.md#troubleshooting).
- Unresolved runtime issue: open a
  [GitHub issue](https://github.com/alireza787b/px4xplane/issues) with the
  X-Plane `Log.txt`, PX4 terminal output, active `config.ini`, and any ULog.

## Current Release Policy

- Use the `v4.0.5` package with the refreshed PX4 `px4xplane-sitl-validation`
  branch while the official PX4 PR is under review.
- PX4 integration PR:
  [PX4-Autopilot #22493](https://github.com/PX4/PX4-Autopilot/pull/22493).
- Temporary helper behavior: until #22493 merges, `setup_px4_sitl.sh` and
  plain `px4xplane` use the maintained `alireza787b/PX4-Autopilot-Me`
  `px4xplane-sitl-validation` branch and sync it by default. Use `--no-sync`
  only for deliberate offline/debug runs.
- Separate PX4 EKF edge-case PR:
  [PX4-Autopilot #27533](https://github.com/PX4/PX4-Autopilot/pull/27533).
  This covers the fast-VTOL transition EKF-GSF yaw-reset issue found during
  final Alia validation and is intentionally separate from the plugin package.
- Separate PX4 Standard VTOL handoff PR:
  [PX4-Autopilot #27601](https://github.com/PX4/PX4-Autopilot/pull/27601).
  This covers the front-transition pusher-thrust and pitch-setpoint handoff
  issue found during Alia validation and is intentionally separate from the
  plugin package.
- For local validation while #27533 and #27601 are pending, run
  `px4xplane --validation --reset-config`. The launcher keeps the validation
  branch current, uses the Tailsitter and TECS guards already included in that
  branch, and stacks the EKF-GSF and Standard VTOL guard commits locally. Use
  `px4xplane --without-ekf-gsf-guard` or
  `px4xplane --without-vtol-handoff-guard` only when selectively disabling
  those two cherry-picked guards. Use `px4xplane --exact-pr` for exact-branch
  testing without temporary guards.
- For reviewer-scope checks of the exact PX4 X-Plane PR branch without
  validation-only tuning or guard commits, run `px4xplane --exact-pr`.
- If you already maintain an official PX4 checkout, use
  `px4xplane --px4-path ~/PX4-Autopilot --sync --reset-config`. The launcher
  adds/updates a separate `px4xplane` remote and does not rewrite `origin`.
- Use `px4xplane --px4-path ~/PX4-Autopilot --restore-official` to reset that
  checkout back to official PX4 `master` after validation.
- The plugin runtime config is `px4xplane/64/config.ini`.
- `config_schema.json` is editor and validator metadata only; it is not read by the plugin at runtime.
- Use `Plugins > px4xplane > Advanced > Open Config Editor` for schema-backed editing and export a new `config.ini` when changes are complete.
- Use `Plugins > px4xplane > Advanced > Validate Config` after changing the active mapping in-sim.
- Confirm the X-Plane aircraft, active `config_name`, and PX4 `SYS_AUTOSTART` target match before arming.
- Run `px4xplane --sync --reset-config` before release validation when you want
  a fully fresh prompt/config cycle. Plain `px4xplane` still syncs the selected
  PX4 checkout, updates submodules, and clears stale SITL parameters.

## Public Docs Policy

Internal analysis reports and scratch notes are intentionally not part of the public docs index or release package. Keep durable user-facing material in the guides above, and keep long-form investigation history outside release artifacts unless it is rewritten as a stable guide.
