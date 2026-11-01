# Next Ref6 Debug Test

Use the SAME debug folder and installed Ref6 aircraft. The one-time bootstrap
upgrade command is supplied in Telegram; after that, updates use run.sh update.
This profile repairs missing MC-yaw/FW-roll allocation and corrects the hover
seed. It is NOT a production tune or a hardware configuration.

## 1. Update, Then Configure Upload Once

PX4 must be stopped. X-Plane can remain open for this parameter-only update.

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
bash run.sh configure-upload
```

Paste the private upload-only token from Telegram at the hidden prompt. Do this
only once, not every flight. Never include the token in a screenshot or log.
For subsequent tests run only the update command, then the flight command below.

## 2. Prepare X-Plane and QGC

- Load the SAME Aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf.
- Select QuadTailsitterRef6 under Plugins > PX4 X-Plane > Airframes.
- Set no wind for the first corrected test.
- Start TruthCapture and leave X-Plane unpaused. It may KEEP recording between
  runs: each shutdown snapshots only its own current-run window.
- Open Windows QGC (UDP 14550) and choose Connect to SITL in the plugin.

## 3. Launch From WSL

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh flight --yes-debug
```

The runner reuses ~/PX4-Autopilot-Me, automatically detects the Windows gateway
for WSL2 NAT and resolves your installed truth-output path. It prints all values.
Override --host and --qgc for mirrored networking/separate computers. It pins the
same official baseline plus logging-only diagnostics, not a new PX4 control fix.
Reconnect the plugin if the build outlasts its connection wait.

## 4. PX4 Console, One Command at a Time

Wait for REF6 PARAMETER GATE PASSED and Ready for takeoff. Confirm QGC connected.

```text
commander check
commander takeoff
```

Use THIS console takeoff, not a different QGC altitude request, so the commanded
altitude is unambiguous: 180 m. At 2 m/s, the climb itself takes approximately
90 seconds. The earlier prolonged ground delay is a separate hover-seed issue.
Observe 20 seconds of controlled hover before continuing. Abort if yaw/roll
oscillations grow, the estimator warns persistently, or the aircraft is unstable.

```text
commander transition
```

Observe at least 20 seconds of controlled FW flight. Do not change parameters.
Only while controlled with adequate terrain clearance, request MC:

```text
commander transition
```

Wait for upright recovery and 20 seconds of stable hover, then:

```text
commander land
```

Require actual descent. If it still climbs or cannot settle, stop and preserve
that failure; do not continue an uncontrolled recovery. After touchdown/disarm:

```text
shutdown
```

The runner automatically snapshots truth, archives and uploads. Do NOT stop the
recorder or press Enter for collection. Wait for UPLOAD VERIFIED, then tell the
reviewing agent the test finished. If upload fails, the ZIP stays local:

```bash
bash run.sh retry-uploads
```

To test offline explicitly: add --no-upload to the flight command. To disable
uploads for production preparation: bash run.sh disable-upload. No chat or
unreviewed evidence is deleted automatically. See COMMANDS.md for extra status
commands and README.md for rollback and private debug-only behavior.
