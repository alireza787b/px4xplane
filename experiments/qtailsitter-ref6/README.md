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

The old 6 kg model settled near 0.71 normalized hover command. The compact
candidate is 2 kg with 0.60 m projected span, 0.10 m chord and four 500 W shaft
power caps. Plane Maker's power field is horsepower, not kW. The hover seed is
now 0.26 from the successful r6 2 kg hover/GoTo/Land log. Original motor
orientations/rotation signs and official rotor direction vectors are retained.
R7's two-GoTo/RTL capture confirms better MC tracking: first-leg pitch-rate
error approximately 0.042 rad/s RMS, with 2.3/3.0 m along-track overshoots.
R8 failed FW/back recovery: FW bank oscillated around 0.4 Hz; the back-transition
ended by timer while still pitched -52.8 degrees; subsequent MC roll/position
oscillation repeatedly triggered emergency braking and interrupted landing.
R9 changed MC_ROLLRATE_P from 0.031815 to 0.060 and completed moderate MC flight,
RTL and landing with zero emergency-braking activations. Its second GoTo still
showed weak yaw tracking; it did not validate recovery from a large disturbance.
R10 keeps R9 MC gains and changes only FW_RR_P from 0.07536 to 0.10. Other FW
gains remain frozen, including FW_YR_P: do not automatically retune FW yaw from
MC roll. Next test measures 15 seconds FW, then normal back-transition and Land.
These are experimental parameters, not a full-flight stability claim. See REPORT.md.
CG is 5 mm ahead of a wing-only quarter-chord estimate. Secondary panels are
disabled explicitly. Component inertia informs initial rate gains, but X-Plane
automatic inertia remains enabled because its manual serialization is unverified.
Other MC gains and all speed/jerk/position limits remain at the r7 values.
FW compression/filter/collective differences limit normalized PID equivalence.
Read design.json and sizing.json for assumptions and envelope screening. Real
control authority, trim, stability and landing remain simulator validation gates.

## Daily Commands

From the same installed debug folder:

```bash
bash run.sh update
bash run.sh flight --yes-debug
```

Follow QUICKSTART.md: console takeoff to 180 m, settled hover, front transition,
15 seconds established FW, normal back-transition, settled MC, then Land at
current position. Do not use RTL or new GoTos in this measurement. The optional
hover scenario remains available for MC lateral tests and separately records
MPC_YAW_MODE=5. Each isolated rootfs starts from defaults; no production or
saved airframe defaults are changed.
Update checks local managed-file hashes, downloads a checksum-verified manifest
and ZIP, and updates in place with one rollback journal. It preserves
flight runs and credentials. Local edits are not discarded. PX4 must be stopped.
For ACF changes X-Plane must also be closed: the same installed ACF and its
receipt are updated together with managed files under a rollback journal.
Local aircraft edits or an inconsistent receipt stop the update. One prior
managed version, including replaced installed bytes, is retained for recovery.

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
