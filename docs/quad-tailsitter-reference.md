# Experimental QuadTailsitter Reference

This is a separate SITL experiment, not a replacement for any supported aircraft
and not a hardware parameter set. No flight-quality or hardware-safety claim is
made before the recorded acceptance tests pass.

## Reproduce the Model

From the px4xplane source directory:

```bash
python3 tools/build_qtailsitter_reference.py \
  aircraft/QuadTailsitter/QuadTailsitter.acf \
  "$HOME/PX4_QTAILSITTER_REF6_2026-11-01"
python3 -m unittest discover -s tests -p 'test_qtailsitter_reference.py' -v
```

The builder refuses to overwrite an existing generated aircraft. It produces a
separate `aircraft/QuadTailsitterRef6` directory, `profile.sh`, `parameters.json`
with parameter rationale, and `design.json` with source/output hashes and ACF
changes. The original aircraft, installed plugin and PX4 checkout are unchanged.
The delivered test bundle adds the reversible installer and evidence runner.

Its maintained source lives in `experiments/qtailsitter-ref6`. To generate a
complete bundle in fresh directories, rather than manually copying old scripts:

```bash
python3 tools/package_qtailsitter_reference.py \
  --output-root /tmp/ref6-package --archive-dir /tmp/ref6-archives
```

The packager refuses existing outputs and writes an archive checksum plus
per-file SHA256SUMS. `--review-dir` can attach a reviewed report/evidence folder;
it never includes original large flight logs automatically.

## Design Contract

| Item | Reference candidate | Reason or limitation |
| --- | --- | --- |
| Loaded mass | 6 kg, no default payload | Distinguish actual mass from ACF maximum weight |
| Main panels | 0.331 m2; existing span and sweep | Approximately preserve recorded baseline wing loading |
| Chord placement | Existing quarter-chord lines retained | Avoid introducing an accidental aerodynamic-center shift |
| Rotors | Four uncanted, counter-rotating | ACF and allocation share vertical axes and measured hub positions |
| Propellers | Geometric 10x8-inch, two-blade candidate | Not a manufacturer replica; thrust/torque calibration pending |
| Power | 750 W shaft per motor | 3 kW aggregate; approximately 3.58 kW electrical at assumed 85% efficiency plus 50 W auxiliary |
| RPM | 13,000 ceiling requested through redline and FADEC | Must verify actual RPM remains below ceiling in X-Plane |
| Propulsion response | 0.1 s engine-only free-rev seed | Not a loaded time constant; measure acceleration and deceleration |
| Prop material | 1,400 kg/m3 composite-density proxy | Declared simulation assumption, not measured prop inertia |
| Airframe inertia | X-Plane automatic calculation | Manual zero fields do not mean zero physical inertia; runtime verification pending |

The four existing rotor arms are retained. Halving all distances would make
10-inch propeller disks overlap. Increasing mass while halving span with the
same chord would also increase wing loading and stall speed substantially.

The sizing analyzer now sums individually serialized panels once. Its legacy
`mirrored_area_m2` JSON key remains available but no longer creates extra panels.
`selected_area_m2` uses the four main panels; secondary surfaces are reported
separately. This convention is specific to this QuadTailsitter analysis, not a
universal ACF wing-classification rule.

## Calibration and Tuning Order

1. Verify actual aircraft identity, mass, prop clearance and force/moment signs.
2. Compare actual RPM, thrust and shaft torque/power against reference data,
   including forward-flight advance ratio. Static thrust alone is insufficient.
3. Measure loaded spin-up and spin-down. A correlation peak in a closed-loop
   flight is not an independently identified actuator delay.
4. Tune rate loops, attitude loops, navigation and finally TECS, in that order.
   Preserve the distinction between MC and FW axes on a tailsitter.
5. Confirm both transitions and Land. Do not disable emergency braking or relax
   EKF gates to make the acceptance test appear successful.

The generated rate gains start from the known quiet-hover baseline. Hover thrust
is now the measured Ref6 normalized-thrust seed, 0.71, not a shaft-power fraction.
All four common CT scales are explicitly 6.5, matching PX4's default common
allocation scale: with KM approximately 0.01905, CT=2 fell below the allocator's
0.05 weak-axis cutoff and removed MC-yaw/FW-roll control. The pinned-source host
test reproduces that cutoff and verifies common-scale normalization and restored
controllability. This is a self-introduced configuration error, not a new PX4 bug.
Cruise and transition settings remain explicit provisional seeds, not a completed
system identification. The bundle records effective parameter history because
the ULog initial snapshot may precede startup overrides.

An earlier flight selected Land but retained emergency braking and climbed.
The latest Ref6 flight instead ended in quad-chute with the missing control axis.
Neither validates a commanded normal back transition or landing.

## Private Debug Loop

Keep one bundle folder and one Ref6 aircraft. `run.sh update` verifies and updates
managed files in place with one rollback journal; it preserves receipts and runs.
`run.sh configure-upload` stores the owner's private token once outside the
bundle. `run.sh flight --yes-debug` resolves the WSL2 NAT host and truth root.
After shutdown, it snapshots complete truth rows without stopping the recorder,
archives the closed ULog and uploads to the private bounded server. A verified
checksum receipt is required; failed transfers remain pending locally.
Use `retry-uploads`, `disable-upload` and `--no-upload` as documented. None of this
changes the production plugin or official PX4 startup. Never ship credentials or
enable this private uploader in a production release.

## Sources and Limits

- [APC 10x8E analytical performance](https://www.apcprop.com/files/PER3_10x8E.dat):
  reference thrust and power, not measured certification data.
- [APC RPM limits](https://www.apcprop.com/technical-information/rpm-limits/):
  the Thin Electric and Multi-Rotor product categories have different limits.
- [Plane Maker manual](https://developer.x-plane.com/manuals/planemaker/):
  engine-only free-rev time, propeller inertia and FADEC settings.
- [PX4 rate-loop tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter).
- [Differential-thrust tailsitter research](https://arxiv.org/abs/1903.06393):
  supports identifying response before selecting controller gains; its gains
  and structural notch filters are not transferable to this model.

The server cannot run the user's Windows X-Plane instance. Structural and host
tests do not substitute for the instrumented simulator flight. No PX4 core
change or public release is justified solely by this candidate model.
