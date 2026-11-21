# px4xplane Documentation

## Start Here

- [README](../README.md#quick-start) - installation and beginner quick start
- [Beta / Developer Launcher](guided-launcher.md) - optional guided setup and maintained airframe settings
- [Custom Airframe Config](custom-airframe-config.md)
- [Config Editor](config-editor.html)
- [Config Schema](developer/config-schema.md)

The current published package is `v4.2.1`. Normal setup uses native official
PX4 targets as described in the README. The optional beta launcher uses the
project's maintained branch for the latest airframe settings. Its `--official`
mode selects bare upstream without those project updates. See
[source modes](guided-launcher.md#source-modes) before comparing results.

## Flight-Test Cards

These checks use the maintained project's airframe settings via the beta
launcher. They are not a claim that the same defaults are already upstream.

- [Alia 250](ALIA_XPLANE12_TEST.md)
- [Ehang 184](EHANG184_XPLANE12_TEST.md)
- [QuadTailsitter](QUADTAILSITTER_XPLANE12_TEST.md)
- [Cessna 172](CESSNA172_XPLANE12_TEST.md)
- [TB2](TB2_XPLANE12_TEST.md)

## Native PX4 Setup

For the standard workflow on your platform, follow the official
[PX4 development environment](https://docs.px4.io/main/en/dev_setup/dev_env.html)
and [X-Plane simulation guide](https://docs.px4.io/main/en/sim_xplane/).
The beta launcher is an alternative for project tuning and development.

## Network and Platform Notes

PX4 connects to the X-Plane plugin on TCP `4560`. When running PX4 in WSL2,
Docker or another computer, the launcher detects or prompts for the host.
For a manual PX4 launch, set it before the `make` command:

```bash
export PX4_SIM_HOSTNAME=<xplane-host-ip>
```

Allow inbound TCP `4560` on that host. Same-machine Linux/macOS or mirrored
WSL2 normally uses localhost; WSL2 NAT needs the Windows host address.
The launcher detects WSL networking and refreshes automatically detected
addresses; manually entered addresses remain saved until `--reset-ip`.

QGC telemetry is separate: normally UDP `14550` on the QGC host. Native PX4
users running QGC on Windows should follow the official
[WSL QGC connection instructions](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl#qgroundcontrol-on-windows).
The beta launcher targets Windows QGC directly; use
`px4xplane --qgc-host <qgc-host-ip>` for a separate QGC machine.
A reported target address is not confirmation
that QGC is receiving; check for a connected vehicle in QGC. See the
[launcher networking guide](guided-launcher.md#x-plane-and-qgc-networking)
for the optional, explicitly enabled MAVLink Router path.

## Troubleshooting

- Wrong X-Plane host: run `px4xplane --reset-ip` and select the correct address.
- QGC has no vehicle: check its host address, UDP `14550`, firewall and QGC
  connection status separately from the X-Plane TCP connection.
- Beta launcher update paused for local edits: choose whether to save ordinary
  files aside or keep them in place. Edited submodules, embedded repositories
  and unfinished Git operations need individual review; no work is discarded.
  See [checkout recovery](guided-launcher.md#existing-checkouts-and-sync).
- Unexpected saved tuning: inspect the selected source, actual PX4 ROMFS
  airframe file and runtime state. Plugin reference files are not executed.
  The launcher offers keep/reset when saved parameters exist; Enter keeps them.
  `px4xplane --airframe <target> --reset-params` backs up and resets only that runtime.
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
- Actuator-feedback or sensor-queue fault: confirm the selected PX4 source,
  restore packaged `actuator_feedback`, reduce X-Plane load and reconnect.
- Mapping problem: use `Plugins > PX4 X-Plane > Advanced > Validate Config`
  or `python3 tools/validate_config.py config/config.ini` from a source checkout.
- Build problem: see [Build Guide troubleshooting](BUILD.md#troubleshooting).
- Unresolved issue: report sanitized X-Plane `Log.txt`, PX4 output, active
  `config.ini` and relevant ULog details in a
  [GitHub issue](https://github.com/alireza787b/px4xplane/issues).

## Developer Validation

- [Source Modes](guided-launcher.md#source-modes) - maintained defaults and bare upstream
- [Build Guide](BUILD.md)
- [Developer Guide](DEVELOPER.md) - development, CI and release policy
- [Airframe Reference Contract](DEVELOPER.md#airframe-reference-contract)
- [Verification Limits](DEVELOPER.md#verification-limits)

Before arming, match the loaded aircraft, active `config_name` and PX4
`SYS_AUTOSTART`. The plugin reads `px4xplane/64/config.ini`; `config_schema.json`
is editor/validator metadata, not runtime configuration.

TruthCapture is optional diagnostic recording. Neither native PX4 setup nor
the beta launcher requires additional recording tools.

## Public Docs Policy

Keep durable setup and operating guidance here. Internal investigation notes
and private evidence are not part of the public documentation or release kit.
