# px4xplane Documentation

## Start Here

- [README](../README.md#quick-start) - installation and beginner quick start
- [Official PX4 Development Environment](https://docs.px4.io/main/en/dev_setup/dev_env.html)
- [Official PX4 X-Plane Simulation](https://docs.px4.io/main/en/sim_xplane/) - targets and run commands
- [Optional Guided Launcher](guided-launcher.md) - automatic setup, checkout safety and saved parameters
- [Custom Airframe Config](custom-airframe-config.md)
- [Config Editor](config-editor.html)
- [Config Schema](developer/config-schema.md)

The current package is `v4.2.1`. Normal runs use official PX4 `main`; the
launcher does not require experimental patches.

## Flight-Test Cards

- [Alia 250](ALIA_XPLANE12_TEST.md)
- [Ehang 184](EHANG184_XPLANE12_TEST.md)
- [QuadTailsitter](QUADTAILSITTER_XPLANE12_TEST.md)
- [Cessna 172](CESSNA172_XPLANE12_TEST.md)
- [TB2](TB2_XPLANE12_TEST.md)

## Network and Platform Notes

PX4 connects to the X-Plane plugin on TCP `4560`. When running PX4 in WSL2,
Docker or another computer, set the X-Plane host before the official `make`
command:

```bash
export PX4_SIM_HOSTNAME=<xplane-host-ip>
```

Allow inbound TCP `4560` on that host. Same-machine Linux/macOS or mirrored
WSL2 normally uses localhost; WSL2 NAT needs the Windows host address.
The launcher detects WSL networking and refreshes automatically detected
addresses; manually entered addresses remain saved until `--reset-ip`.

QGC telemetry is separate: normally UDP `14550` on the QGC host. For remote
QGC, use `px4xplane --qgc-host <qgc-host-ip>` and allow that UDP traffic.
Direct MAVLink is the default. A reported target address is not confirmation
that QGC is receiving; check for a connected vehicle in QGC. See the
[launcher networking guide](guided-launcher.md#x-plane-and-qgc-networking)
for the optional, explicitly enabled MAVLink Router path.

## Troubleshooting

- Wrong X-Plane host: run `px4xplane --reset-ip` and select the correct address.
- QGC has no vehicle: check its host address, UDP `14550`, firewall and QGC
  connection status separately from the X-Plane TCP connection.
- Checkout update refused: inspect `git status` in PX4 and its submodules.
  Preserve your changes yourself, or select another clean checkout with
  `--px4-path`. The launcher does not force-stash, reset or clean your work.
- Unexpected saved tuning: inspect the selected airframe's state directory.
  `px4xplane --reset-params` backs up and resets only that selected runtime.
  Syncing PX4 or changing defaults does not automatically delete tuning.
  `--reset-config` resets launcher choices, not PX4 parameters.
- Missing launcher command: rerun the setup helper in the
  [guided launcher guide](guided-launcher.md#install-and-run). Use `--repair`
  for dependency/setup repair when the command already exists.
- Plugin missing: confirm the complete `px4xplane` folder is under X-Plane's
  `Resources/plugins`, then check X-Plane `Log.txt` for load errors.
- High accelerometer bias or unstable stationary velocity: close X-Plane,
  replace the complete plugin folder and confirm `Log.txt` reports `v4.2.1`
  and `FLOW:actuator_feedback`. Do not raise `EKF2_ABL_LIM` or calibrate PX4
  level to mask a timing fault. Inspect saved parameters before any explicit
  backed-up reset.
- Actuator-feedback or sensor-queue fault: use current official PX4 `main`,
  restore packaged `actuator_feedback`, reduce X-Plane load and reconnect.
- Mapping problem: use `Plugins > PX4 X-Plane > Advanced > Validate Config`
  or `python3 tools/validate_config.py config/config.ini` from a source checkout.
- Build problem: see [Build Guide troubleshooting](BUILD.md#troubleshooting).
- Unresolved issue: report sanitized X-Plane `Log.txt`, PX4 output, active
  `config.ini` and relevant ULog details in a
  [GitHub issue](https://github.com/alireza787b/px4xplane/issues).

## Developer Validation

- [Guided Launcher Validation](guided-launcher.md#developer-validation) - explicit TECS #27670 testing and custom checkout flags
- [Build Guide](BUILD.md)
- [Developer Guide](DEVELOPER.md) - development, CI and release policy
- [Compact QuadTailsitter Experiment](compact-qtailsitter.md) - opt-in model/profile development

Before arming, match the loaded aircraft, active `config_name` and PX4
`SYS_AUTOSTART`. The plugin reads `px4xplane/64/config.ini`; `config_schema.json`
is editor/validator metadata, not runtime configuration.

## Public Docs Policy

Keep durable setup and operating guidance here. Internal investigation notes
and private evidence are not part of the public documentation or release kit.
