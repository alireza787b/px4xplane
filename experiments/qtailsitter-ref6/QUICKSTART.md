# Next Test A: 10 m Hover And Land

SITL only. Same folder, same QuadTailsitterRef6 aircraft. No new PX4 control
patch or plugin binary. No transition in this test. See REPORT.md for changes.

## 1. Close X-Plane And Update In WSL

Stop PX4, then:

```bash
cd "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
bash run.sh update
cat VERSION.txt
```

Expected: `2026-11-01.ref6.6`. The installed ACF updates automatically; do not
copy another aircraft or edit config.ini. Your existing upload setup is kept.
If the updater reports local edits, send that error rather than deleting files.

## 2. Open X-Plane

- Load Aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf; no wind.
- Check Weight and Balance: total 2 kg, payload zero. Do not add ballast.
- Start TruthCapture recording.
- Plugins > PX4 X-Plane > Airframes > QuadTailsitterRef6, then Connect to SITL.
- Open Windows QGC if wanted. Use the PX4 console below for the same test target.

## 3. Start Test A In WSL

```bash
bash run.sh A
```

It reuses ~/PX4-Autopilot-Me and detects your installed X-Plane/truth paths.
Reconnect the plugin if building outlasts its wait. No token setup again.

Wait for REF6 PARAMETER GATE PASSED and Ready for takeoff. In PX4, one at a time:

```text
commander check
commander takeoff
```

At 10 m, observe 20 seconds ONLY if controlled. No transition. Then:

```text
commander land
```

After touchdown and automatic disarm:

```text
shutdown
```

Leave TruthCapture recording. Logs and truth snapshot upload automatically.
Wait for UPLOAD VERIFIED, then tell me the test finished. No manual ZIP needed.
For a failed upload: `bash run.sh retry-uploads`.

If attitude diverges, stop the simulation and preserve the failed interval with
shutdown. Do not force arm or increase EKF limits. Bench NOT VERIFIED means
measurements remain pending, not a measured failure. This is not a flight-ready
or hardware profile. Full-flight steps remain in COMMANDS.md for later testing.
