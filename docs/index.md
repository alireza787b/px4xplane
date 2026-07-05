# px4xplane Documentation

This is the stable documentation entry point for the PX4-XPlane plugin,
packaged aircraft, setup helper, and configuration tools.

## Start Here

- [README](../README.md) - beginner quick start and release download
- [Official PX4 X-Plane Simulation](https://docs.px4.io/main/en/sim_xplane/)
- [Official PX4 Development Environment](https://docs.px4.io/main/en/dev_setup/dev_env.html)
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

Also allow inbound TCP `4560` on the X-Plane host firewall. The `px4xplane`
launcher can auto-detect common WSL2 host IPs, and
`px4xplane --reset-ip` clears a saved wrong IP and asks again.

## Troubleshooting

- Wrong X-Plane host IP: run `px4xplane --reset-ip` and choose the correct host
  IP again.
- Stale local PX4 checkout, submodules, or SITL parameters: run `px4xplane`.
  The launcher syncs the selected PX4 branch by default and clears stale SITL
  parameter files when airframe defaults change. Use
  `px4xplane --sync --reset-config` when you also want to clear remembered
  IP/config choices.
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

## Validation and Maintainer Notes

- Use the `v4.0.7` package with official PX4 `main`. X-Plane SITL support is
  merged in
  [PX4-Autopilot #22493](https://github.com/PX4/PX4-Autopilot/pull/22493).
- The launcher uses official PX4 `main` as the base. By default it asks whether
  to layer the pending validation PRs locally. Use `--official` for a clean
  official-PX4 run without pending validation PRs. Use `--no-sync` only for
  deliberate offline/debug runs.
- Separate PX4 EKF edge-case PR:
  [PX4-Autopilot #27533](https://github.com/PX4/PX4-Autopilot/pull/27533).
  This covers the fast-VTOL transition EKF-GSF yaw-reset issue found during
  final Alia validation and is intentionally separate from the plugin package.
- Separate PX4 Standard VTOL handoff PR:
  [PX4-Autopilot #27601](https://github.com/PX4/PX4-Autopilot/pull/27601).
  This covers the front-transition pusher-thrust and pitch-setpoint handoff
  issue found during Alia validation and is intentionally separate from the
  plugin package.
- Separate PX4 fixed-wing TECS altitude-frame PR:
  [PX4-Autopilot #27670](https://github.com/PX4/PX4-Autopilot/pull/27670).
  This covers stale altitude-reference state in fixed-wing TECS.
- Active PX4 Tailsitter attitude-conversion PR:
  [PX4-Autopilot #27793](https://github.com/PX4/PX4-Autopilot/pull/27793).
  This supersedes the earlier local tailsitter frame-guard validation branch.
- For local validation while #27533, #27601, #27670, and #27793 are pending, run
  `px4xplane --validation --reset-config`. The launcher syncs official PX4
  `main` and cherry-picks those PR heads locally for the run. Use
  `px4xplane --without-ekf-gsf-guard`,
  `px4xplane --without-vtol-handoff-guard`,
  `px4xplane --without-tecs-alt-guard`, or
  `px4xplane --without-tailsitter-fw-frame-guard` only for selective A/B tests.
- For a clean official-PX4 baseline without pending PRs, run
  `px4xplane --official --reset-config`.
- If you already maintain an official PX4 checkout, use
  `px4xplane --px4-path ~/PX4-Autopilot --sync --reset-config`. The launcher
  adds/updates a separate `px4xplane` remote and does not rewrite `origin`.
- Use `px4xplane --px4-path ~/PX4-Autopilot --restore-official` to reset that
  checkout back to official PX4 `main` after validation.
- The plugin runtime config is `px4xplane/64/config.ini`.
- `config_schema.json` is editor and validator metadata only; it is not read by the plugin at runtime.
- Use `Plugins > PX4 X-Plane > Advanced > Open Config Editor` for schema-backed editing and export a new `config.ini` when changes are complete.
- Use `Plugins > PX4 X-Plane > Advanced > Validate Config` after changing the active mapping in-sim.
- Confirm the X-Plane aircraft, active `config_name`, and PX4 `SYS_AUTOSTART` target match before arming.
- Run `px4xplane --sync --reset-config` before release validation when you want
  a fully fresh prompt/config cycle. Plain `px4xplane` still syncs the selected
  PX4 checkout, updates submodules, and clears stale SITL parameters.

## Public Docs Policy

Internal analysis reports and scratch notes are intentionally not part of the public docs index or release package. Keep durable user-facing material in the guides above, and keep long-form investigation history outside release artifacts unless it is rewritten as a stable guide.
