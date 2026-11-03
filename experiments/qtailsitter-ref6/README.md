# QuadTailsitter Debug Workflow

Private experimental SITL only. Same Ref6 aircraft and installed debug folder.
No production plugin changes, private dive feature or hardware image.
**Current instructions: QUICKSTART.md.**

## Current Test: R15

The test retains official PX4 main snapshot `649929706fb34bf0b06dbd2df24e04d5d452d616`
plus log-only diagnostics `58794523a151fa33a28a564454454b8c9fbfcdd5`.
The upstream emergency-braking follow-up [#28350](https://github.com/PX4/PX4-Autopilot/pull/28350)
merged on October 12; the old September base did not contain it. That finding
supersedes R12's tune-first recommendation. REPORT.md explains the correction
and the final merged implementation, not the stale PR-body design.

Current upstream also includes TECS altitude/speed and underspeed/bank handling
improvements, and true-airspeed turn coordination. These are not new custom
patches. R13 and R14 landed and automatically disarmed, without repeated Land
braking. R14 reduced early FW oscillation but a longer run still showed a
0.46Hz mode. R15 tests FW_RR_P 0.18 -> 0.22 and modest faster MC requests:
cruise5m/s, climb2.5m/s, descent1.6m/s. MC PID, I/D, attitude gains, NPFG,
trim, FW speeds, mapping, mass/CG and ACF remain unchanged. Near-ground Land
speed stays0.9m/s. The trajectory profile retains
MPC_JERK_AUTO=4, MPC_ACC_HOR=2 and MPC_ACC_HOR_MAX=2. The last is not an auto
emergency-braking limit. R13 was a combined firmware/profile trial, not an
isolated A/B proof of one PR. R15 changes speed bounds and therefore braking
thresholds too; it is not an independent braking fix or a stability promise.
Back-transition still climbs about37m; R14 exited by angle rather than timeout,
but recovery and full-envelope performance remain acceptance gaps.

The unchanged ACF is 2 kg with 0.60 m projected span, 0.10 m chord and four
500 W shaft-power caps. It retains original motor signs and PX4's direction
vectors. CT=6.5 preserves the modeled yaw/FW-roll allocation above the weak-axis
cutoff; it is a normalized allocation scale, not measured maximum thrust.
Prop/cant effectiveness, runtime inertia and FW trim remain validation tasks.

R11's quiet initial hover did not establish recovery stability. It returned to
MC around 9.3 m/s, repeatedly released emergency braking with substantial
trajectory acceleration retained, and interrupted Land descent. A separate
0.448 Hz FW roll oscillation occurred earlier. Late power collapse near 800 s
is strongly consistent with battery depletion, not the early failure.
See REPORT.md and the packaged R13 plots for current measured evidence.

## Daily Commands

Stop PX4 and the old TruthCapture recording:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
bash run.sh prepare-truth
```

Expect2026-11-03.ref6.15. Load/reset a fresh charged Ref6 FIRST, no wind/payload,
THEN start a NEW TruthCapture recording, QGC and plugin connection. Don't reload
or reset the flight/recorder after that. R14's reset-invalid truth window was
rejected, not silently stitched. Then:

```bash
bash run.sh flight --yes-debug
```

Follow QUICKSTART.md: console takeoff180m, settled hover, short MC GoTo/stop and
vertical checks, transition,75s clean FW window, back-transition, decaying recovery,
Land, touchdown/disarm,
shutdown. No FW GoTo/retarget, RTL, speed commands, autotune or gain edits.
Stop growing/non-decaying recovery within 25 seconds; do not waste minutes of
battery/evidence. Hold can command translation toward its old target; Land
should stop this travel. Limit airborne time to eight minutes.

The legacy recovery-jerk case remains compatible but now duplicates baseline
jerk=4. braking-isolation remains an explicit diagnostic only, not a recommended
test or fix. Neither raises the default maximum speed. Do not use old commands
instead of the current guide.

## Source And Installation

The update downloads a checksum-verified package into the SAME folder, preserves
runs/credentials and retains one previous managed version. It does not discard
local managed-file edits. PX4 must be stopped; X-Plane must also be closed only
if an update changes installed ACF bytes. R15 does not change the ACF/config.

The Git bundle carries current official source history plus the log-only commit.
It reuses ~/PX4-Autopilot-Me; no second PX4 clone. Its prerequisite is the old
September base already used in these tests. Missing submodule objects still
download. The runner aligns submodules to the pinned commits before building,
records identities, and restores original source/submodule identities afterward.
A dirty checkout stops the test instead of discarding user work.

Each isolated rootfs starts fresh with official airframe 5021 and the explicit
Ref6 profile. Startup compares every profile setting, altitude and scenario
override before declaring PARAMETER GATE PASSED. Both firmware pins, ACF/config
hashes and effective scenario are recorded.

## Recorder And QGC

prepare-truth appends a receipted, reversible block to TruthCapture's custom
catalog: native total/prop/aerodynamic moments/forces, runtime unit-mass inertia
and electrical state. It writes no simulator physics and replaces no plugin.
Stop old capture and start a NEW recording afterward. Full-flight tests require
these datarefs to exist before source changes; hover/A retain prior behavior.
Undo with restore-truth while PX4/recording are stopped. User catalog edits are
not overwritten.

The runner derives the current WSL2 Windows gateway/QGC host and truth output
root from the installation receipt. Override --host, --qgc, --truth or --repo for
other layouts. A direct unicast QGC MAVLink instance targets Windows UDP14550;
no router is required. Check the printed host and QGC heartbeat before arming.
qgc-firewall.ps1 contains the scoped Windows rule when needed.

## Automatic Evidence Upload

Existing upload credentials remain outside the bundle at
~/.config/px4xplane-debug/upload.json, mode0600. Do not put a token in Git, a
public download, command arguments or evidence. Only if not yet configured:

```bash
bash run.sh configure-upload
```

After PX4 shutdown, the runner snapshots complete truth rows for THIS run,
restores source, creates a ZIP and uploads automatically. Leave the recorder
running until collection ends. Windows up to 256 MiB remain raw CSV; larger
complete windows stream to frames.csv.gz with a 1 GiB input and 128 MiB compressed
ceiling. The ZIP limit remains 256 MiB. No silent row truncation or decimation.

UPLOAD VERIFIED requires the server's matching checksum and byte count.
Network/quota/auth failures retain a local pending ZIP. Retry:

```bash
bash run.sh retry-uploads
```

After verified upload, tell the reviewing agent the test finished. The archive
is on the analysis upload server, not necessarily in VPS home. No manual
copying is required. Keep original Windows recordings until review completes.

## Disable And Restore

```bash
bash run.sh disable-upload
bash run.sh restore-truth
```

For source restoration after an interrupted run:
`bash run.sh restore --run PATH --repo PATH`.
To restore original plugin selection/config, quit X-Plane then
`bash run.sh restore`. The original aircraft was never replaced.

Maintained source: experiments/qtailsitter-ref6; package with
tools/package_qtailsitter_reference.py. server/README.md documents disabling
the private intake and scoped proxy route at production preparation.
No diagnostic upload is added to production px4xplane. Reports are measured
analysis and AI review, not human maintainer approval or certification.
