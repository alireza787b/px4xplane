# Ref6: Your Next X-Plane Test

This is a NEW, experimental simulation aircraft, not a release or hardware tune.
Original aircraft and normal PX4 settings stay intact. No plugin binary update
is required for this test. The runner adds a separate configuration section.

## 1. Extract the Telegram ZIP

In Windows Downloads, right-click `PX4_QTAILSITTER_REF6_2026-11-01.zip`, choose
**Extract All**, and extract into Downloads. The resulting directory must contain
`PX4_QTAILSITTER_REF6_2026-11-01/run.sh` (avoid a double-nested folder).

Open WSL. Copy this block once:

```bash
test ! -e "$HOME/PX4_QTAILSITTER_REF6_2026-11-01" && cp -a '/mnt/c/Users/Alireza/Downloads/PX4_QTAILSITTER_REF6_2026-11-01' "$HOME/"
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
sha256sum --check SHA256SUMS
bash run.sh check
```

If the home folder already exists, stop: do not mix two versions. Check its
`VERSION.txt` against the delivered version before reusing it.

## 2. Install the Separate Aircraft

**Quit X-Plane first.** Keep QGC closed during installation. In the same WSL:

```bash
export XP='/mnt/h/X-Plane 12'
export CFG="$XP/Resources/plugins/px4xplane/64/config.ini"
test -f "$CFG"
bash run.sh install --xplane "$XP" --config "$CFG"
```

This uses your previously confirmed plugin location. If `test -f` fails, do not
create a new config: find the actual installed plugin config and set CFG to it.
The original aircraft is never replaced. The installer saves a rollback receipt.

## 3. Open the NEW Aircraft and Start Recording

1. Open X-Plane. Load `Aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf`.
2. Set no wind for this first comparison. Disable AI aircraft/control overrides.
3. In Plugins > PX4 X-Plane > Airframes, select **QuadTailsitterRef6**.
4. Start a **new TruthCapture recording**, with X-Plane unpaused.
5. Open Windows QGC. It must listen on UDP 14550.
6. In the PX4 X-Plane menu choose **Connect to SITL**.

## 4. Launch the Test in WSL

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
export XP='/mnt/h/X-Plane 12'
HOST=$(ip -4 route show default | awk 'NR==1 {print $3}')
bash run.sh flight --host "$HOST" --qgc "$HOST" --truth "$XP/Output/XPlaneTruthCapture"
```

This HOST method is for your usual WSL2 NAT setup. The script reuses
`~/PX4-Autopilot-Me`; it does not clone another PX4. It temporarily selects the
pinned, logging-only diagnostic build and restores the prior source afterward.
No private dive feature or other optional firmware fixes are included.
Type `SITL EXPERIMENT` when asked. If the build outlasts the plugin connection
wait, click **Connect to SITL** again. Do not reinstall anything.

## 5. Flight Commands, One at a Time

Wait for `REF6 PARAMETER GATE PASSED` and `Ready for takeoff!`.

```text
ver all
param show MIS_TAKEOFF_ALT
commander check
```

Confirm altitude 180 m and no persistent warning. Then:

```text
commander takeoff
```

Wait until hover has settled at 180 m; observe 20 seconds. **If hover oscillates
or is uncontrolled, skip the transition and collect that failure.** If controlled:

```text
commander transition
```

Wait for FW flight and observe 20 seconds. Use QGC Hold/orbit only if controlled.
Then, while on a shallow settled bank or wings-level with adequate clearance:

```text
commander transition
```

Wait for upright MC recovery and observe 20 seconds. Only if controlled:

```text
commander land
```

Confirm actual descent. If still climbing or not descending within 15 seconds
after settling, stop the experiment and retain evidence. Otherwise let touchdown
and disarm complete. End with:

```text
shutdown
```

For a growing oscillation/dive, pause X-Plane and stop PX4 rather than allowing
another prolonged uncontrolled recovery. Do not relax EKF limits or force arm.
Full checkpoint details and status commands are in [COMMANDS.md](COMMANDS.md).

## 6. Return Evidence

After PX4 exits, **stop TruthCapture**, then press Enter when asked in WSL.
The runner prints the results directory and ZIP path. Upload that ZIP unchanged.

```bash
explorer.exe "$(wslpath -w "$HOME/PX4_QTAILSITTER_REF6_2026-11-01/runs")"
```

Report whether hover, FW, back transition and landing each passed. Do not change
gains midway through this comparison. We need the first failure, not a green
result achieved by changing several unknowns.

## QGC Not Connecting

Do not install mavlink-router yet: this runner sends directly to Windows UDP
14550. In Windows PowerShell, while QGC is open:

```powershell
Get-NetUDPEndpoint -LocalPort 14550 | Select-Object LocalAddress,LocalPort,OwningProcess
```

If no listener appears, enable QGC's UDP auto-connect/listening link. If it is
listening but receives nothing, use the optional `qgc-firewall.ps1` in an
Administrator PowerShell to add a rule scoped to the current WSL IP. It prints
the matching removal command. Do not disable the firewall. Verify vehicle
reception in QGC, not just that PX4 says it is transmitting.

## Restore Your Previous Plugin Selection

Quit X-Plane, then in the bundle directory:

```bash
bash run.sh restore
```

Restores the exact saved config only if it has no post-install edits. Keeps the
Ref6 aircraft folder for inspection; never removes the original aircraft.
