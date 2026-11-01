# QuadTailsitterRef6 Experiment

Separate, unvalidated simulation experiment, not a flight guarantee or hardware
configuration. No automatic arming or transition. This bundle does not fix PX4
core behavior, install a plugin, clone PX4, or access the private dive checkout.
Keep the existing aircraft and normal PX4 parameter stores unchanged.

**Delivered ZIP users: start with [QUICKSTART.md](QUICKSTART.md).** The ZIP already
includes the generated aircraft and profile; no model-building command is needed.
This directory in Git is the maintained runner source. Use
`tools/package_qtailsitter_reference.py` from the repository to generate a bundle.

**Current scope: staged SITL validation, not flight readiness.** `hover` (or `A`)
captures hover only; `flight` permits the checkpointed full scenario in COMMANDS.md.
Both use the same pinned firmware. X-Plane calibration is pending: this VPS has no
X-Plane. The candidate is 6 kg, 0.331 m2, four 0.75 kW uncanted vertical rotors,
provisional 10x8 two-blade geometry, 13,000 RPM ceiling and 0.1 s free-rev setting.
Blade chord scale 2.6 comes from the included reproducible BEM screening. Analytical Q/T about 0.01905 m and PX4
vertical allocation are seeds, not measured force/moment/RPM validation.

Minimal sequence after builder generation: `check`, `install` while X-Plane is
closed, open the new aircraft and start TruthCapture, then `hover`. No measurement
JSON is needed to boot diagnostics or collect the first hover evidence. `bench`
is an optional truthful report of pending/reviewed evidence, not a launch gate.

## 1. Check

The main builder must first supply `aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf`,
`design.json`, `parameters.json`, and `profile.sh`. Missing files stop the workflow. The profile must
contain only comments and literal `param set NAME NUMBER` / `param set-default`
commands plus `param show NAME`. The runner sources a recorded copy and compares
every expected value against `parameters.json`; design ACF hash must also agree.

```bash
cd /home/alireza/PX4_QTAILSITTER_REF6_2026-11-01
python3 -m unittest -v test_ref6
bash run.sh check --repo "$HOME/PX4-Autopilot-Me"
```

Requires Python 3, git, make, pgrep, util-linux `script`, and an existing configured
PX4 build environment. Base `f90b40d61e56405513dfcd8e194287ac60d3a6ae` must already
exist in that checkout. The included diagnostic bundle imports only pinned
`9cfe130a8d8e0c25f88e044141c523247ca53d48`; no stock/tuning fallback. It changes no
source files directly. Build output goes to the existing PX4 build directory;
normal source identities, including initialized submodules, are restored afterward.
Dirty source, another PX4 process, missing files or missing prerequisites: STOP.

## 2. Install With X-Plane Closed

Use your actual Windows install paths, not reference copies. Substitute the correct
plugin `config.ini` path. No recursive disk discovery or default aircraft guessing.

```bash
export XP='/mnt/h/X-Plane 12'
export CFG="$XP/Resources/plugins/px4xplane/64/config.ini"
bash run.sh install --xplane "$XP" --config "$CFG"
```

Linux and, when available, Windows tasklist process checks reject running X-Plane.
If Windows detection is unavailable, installation requires typing `XPLANE CLOSED`
after you quit it. Never confirm while the simulator is running. Detection failure
is not permission to install over a running simulator.

Installs only `Aircraft/QuadTailsitterRef6`, refusing any existing destination.
Copies the original `[QuadTailsitter]` section to `[QuadTailsitterRef6]`, sets its
`aircraftMatch`, and selects it via global `config_name`. Other section values are
preserved, including existing comments and formatting. Only the global selection
and a newly appended section change. Exact
original bytes and hashes are retained under `installation/`. A failed transaction
rolls back its config change and its newly created aircraft folder. Successful
repeat install is a verified no-op. Never delete a partial receipt to force retry.

Plugin section support was verified in existing `src/ConfigManager.cpp`:
`GetAllSections` enumerates names, and `GetValue(configName.c_str(), ...)` and
`parseChannelConfig` resolve selected sections dynamically. No hardcoded Ref6
plugin change is needed. Use a plugin build matching that source; a different
installed binary is not proven compatible by inspecting this repository.

## 3. Open Simulator and QGC

Load **QuadTailsitterRef6.acf** from the NEW folder, not QuadTailsitter.acf. Select
**QuadTailsitterRef6** in the PX4 X-Plane plugin and connect to SITL. Start a NEW
TruthCapture session with X-Plane unpaused. Recorder must be compatible 0.1.9 or
newer 0.1.x and emit `manifest.json`, `frames.csv`, frame/time columns and rows.

The runner requires growing CSV data before checkout changes and again after the
build. Manifest ACF name and resolved loaded path must match the new installed
aircraft; that path's on-disk SHA256 must match both receipt and bundle. An optional
manifest `aircraft_sha256` must also agree. Version 0.1.9 manifests have no ACF hash:
this checks the file at the reported loaded path, NOT X-Plane's in-memory model.
Do not edit/re-save/reload aircraft or change plugin selection during recording.
Stop and restart with a new recording after any such change. Missing/stale truth,
wrong ACF, version or hash mismatch: STOP, do not bypass the gate.

QGC on Windows must listen on UDP **14550**. Runner creates explicit unicast from
WSL UDP **18571** to the IPv4 passed in `--qgc`; no mavlink-router or broadcast
dependency. Give `--host` the X-Plane Windows IPv4. Find addresses on your machine;
do not assume a stale WSL gateway address. Windows Defender Firewall: allow the
QGC executable or inbound UDP 14550, with remote address scope limited to the
current WSL IP. The optional qgc-firewall.ps1 rule covers all profiles because
the WSL virtual interface may be classified Public, but accepts ONLY that IP.
Remove it after testing. Do not disable the firewall, expose WAN ports,
or change router forwarding. Apply equivalent narrow rules for the existing
X-Plane/plugin connection as needed. Confirm reception in QGC, not just a socket.

## 4. Optional BENCH Report

```bash
bash run.sh bench
```

This writes `bench-report.json` with pending force/moment/RPM gates. It does not
start motors or imply calibration. Acquire actual X-Plane BENCH measurements
under a controlled, restrained simulation setup and review motor ordering, spin,
force axes and moments against `parameters.json`. Do not override EKF checks or
force-arm to obtain them. No X-Plane measurements are provided by this VPS.

Pending calibration does NOT prevent a consented SITL evidence-gathering hover.
Do not invent measurements to turn the report green. After collecting evidence,
advanced users can add an optional review JSON; see [BENCH_REVIEW.md](BENCH_REVIEW.md).

## 5. Checkpointed Test

Replace both example IPv4 addresses with your actual Windows addresses.

```bash
HOST=$(ip -4 route show default | awk 'NR==1 {print $3}')
bash run.sh flight --repo "$HOME/PX4-Autopilot-Me" \
  --host "$HOST" --qgc "$HOST" \
  --truth "$XP/Output/XPlaneTruthCapture"
```

Read the warning and type `SITL EXPERIMENT` to consent to this unvalidated test.
The runner builds the same pinned diagnostic firmware, checks loaded aircraft and
live truth, and boots with the isolated profile. At the PX4 prompt follow
[COMMANDS.md](COMMANDS.md): verify the parameter gate and QGC connection, then
`commander takeoff`, the hover checkpoint, and only then the transition scenario.
Abort on tilt growth, motor saturation, wrong yaw response, estimator warnings or
uncontrolled descent. Never force-arm or bypass EKF. **Do not transition unless
hover is controlled.** Replace `flight` with `hover` for a hover-only capture at
50 m rather than 180 m. Full-scenario consent is permission for an unvalidated
simulator experiment, not evidence that calibration or release gates passed.
The gateway-derived HOST is for WSL2 NAT; for mirrored networking or separate
computers provide the actual Windows IPv4 instead.

Fresh runtime and evidence are created under this bundle's `runs/`; airframe remains
**5021**, profile comes only from this bundle. No reuse of normal BSON stores.
Read [COMMANDS.md](COMMANDS.md), one step at a time. No forced arming, no RTL, no
prolonged uncontrolled recovery. Process exit is not evidence of successful flight.

## 6. Collect or Recover

After `shutdown`, stop TruthCapture recording, THEN press Enter. Stable copy must
have grown since preflight. Logs, input identity, profile, source patch and truth
are preserved with checksums and a local ZIP. Nothing is published or sent.
If collection fails, keep originals and retry only once all producers have stopped:

```bash
bash run.sh collect --run "$PWD/runs/flight-REPLACE" \
  --truth "$XP/Output/XPlaneTruthCapture/REPLACE-EXACT-SESSION"
```

If `truth/` already exists omit `--truth`; it is never overwritten. Missing ULog or
truth is explicitly incomplete, even if an archive was produced. Review flight
behavior, timestamps, transitions and GCS reception before drawing conclusions.
If PX4 remains running, source restoration/collection stops. Stop it yourself;
never kill unrelated processes. Retry source recovery using the recorded run:

```bash
bash run.sh restore --repo "$HOME/PX4-Autopilot-Me" --run "$PWD/runs/flight-REPLACE"
```

Source restoration never discards edits or rewinds a moved branch. A dirty checkout
requires manual review. Build artifacts and fetched git objects are retained.

## 7. Restore Original Config

Quit X-Plane first, then:

```bash
bash run.sh restore --state "$PWD/installation"
```

Config-only rollback restores exact original bytes, is idempotent, and refuses
post-install user edits or a corrupt backup. Ref6 aircraft stays on disk; original
aircraft was never overwritten. To reinstall after rollback use a new `--state`
directory AND explicitly archive/move the retained Ref6 aircraft yourself. Keep
receipts/backups. Never remove existing aircraft automatically to force install.

The plugin may re-save INI whitespace when selecting an airframe. Flight checks
allow cosmetic reserialization when all values remain identical. Rollback also
requires all comments to be preserved; meaningful edits are never discarded.
