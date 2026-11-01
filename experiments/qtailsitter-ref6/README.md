# QuadTailsitter Debug Workflow

Private, experimental SITL only. No hardware firmware, production plugin changes,
private dive feature or PX4 control patches are included. See QUICKSTART.md for
the next test. The aircraft and bundle keep the SAME names and locations.

## Current Correction

The previous Ref6 profile retained CT=2 with KM approximately 0.01905. Its
uncanted yaw-effectiveness row was 0.0381, below the recorded PX4 allocator's
0.05 cutoff. That removed MC yaw / FW roll control. The new profile explicitly
uses PX4's default common CT scale 6.5. A host test compiles the recorded PX4
matrix, filter and normalization routines to reproduce the failure and check the
correction. This is a configuration repair, not a newly proven PX4 bug.

The previous hover seed used shaft-power fraction in place of controller
normalized thrust. MPC_THR_HOVER is now 0.71 from settled Ref6 flight evidence;
the thrust estimator remains enabled. Geometry and other gains are unchanged.
The profile is still experimental: real control authority, stability and landing
must pass the next recorded simulator flight.

## Daily Commands

From the same installed debug folder:

```bash
bash run.sh update
bash run.sh flight --yes-debug
```

Update checks local managed-file hashes, downloads a checksum-verified manifest
and ZIP, and updates in place with one rollback journal. It preserves
installation receipts, flight runs and credentials. Local edits are not
discarded. PX4 must be stopped. Updates changing the ACF are rejected for explicit
asset-install review; this correction does not change the ACF.

On WSL2 NAT, the runner detects the current Windows gateway and QGC host. The
TruthCapture output root comes from the recorded X-Plane installation. Override
with --host, --qgc, --truth or --repo for other layouts. Hosts, paths, firmware
revision and upload state are printed and recorded. --yes-debug is explicit
consent to an unvalidated simulation test; it never arms the vehicle.

## One-Time Upload Configuration

```bash
bash run.sh configure-upload
```

Paste the upload-only token supplied privately through Telegram at the hidden
prompt. It is stored in ~/.config/px4xplane-debug/upload.json with mode 0600,
outside the bundle and evidence. Never put it in Git, command-line arguments,
public downloads or a flight report. Tokens expire; renew through the owner when
needed. The endpoint accepts uploads only, not listing or downloads.

After PX4 shutdown, the runner snapshots complete TruthCapture rows for THIS
run, restores source identity, creates a ZIP and uploads it automatically. It
does not stop the recorder or require an Enter press. The summary explicitly
says runner_snapshot, not menu_stop. Replaced/truncated/rewritten captures,
changed session metadata, partial final rows and frame/time resets are checked.
The current-run window is bounded to 256 MiB; source capture files are preserved.

Upload success requires a matching server checksum and byte count. Network,
quota or auth failures retain the local ZIP and pending receipt. Retry with:

```bash
bash run.sh retry-uploads
```

Only say the test finished; the reviewing agent can retrieve the server evidence
through authorized SSH. No manual ZIP copying is needed after verified upload.

## Disable and Restore

```bash
bash run.sh disable-upload
bash run.sh flight --no-upload
```

These affect this debug runner only. Production px4xplane does not auto-upload.
For production preparation, disable the private intake service and its scoped
proxy route as documented in the repository's server/README.md.

To restore source after an interrupted run: run.sh restore --run PATH --repo PATH.
To restore the original plugin selection/config: quit X-Plane, then run.sh restore.
Rollback refuses meaningful user edits. The original aircraft is never replaced.
Use run.sh check for bundle, profile and diagnostic identity checks.

Maintained source: experiments/qtailsitter-ref6. Package with
tools/package_qtailsitter_reference.py. The server implementation is deliberately
not shipped in the client ZIP. Reports and plots describe recorded flights, not
certification or a human PX4 maintainer's sign-off.
