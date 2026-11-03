# Compact QuadTailsitter: Draft Model Migration

The revised compact profile is a 2 kg, 0.6 m span, four-motor tailsitter without
control surfaces. It uses differential thrust, a body-axis virtual pitot and
the existing PX4 target `xplane_qtailsitter` (`SYS_AUTOSTART=5021`).

**This is draft work.** The previous 5 kg model and the compact model are NOT
interchangeable. The matching compact QuadTailsitter model is supplied privately by
its owner; it is not included in this repository, CI artifacts or releases.
Do not download a different aircraft and assume these parameters apply.
Companion PX4 draft: [PX4 #29598](https://github.com/PX4/PX4-Autopilot/pull/29598).
Do not treat an open draft as an already-merged official profile.

## Model, Mapping And Parameters

| Component | Compact model pairing |
| --- | --- |
| X-Plane file | Owner-supplied model; use the exact path in its private installation guide |
| Plugin section | `[QuadTailsitter]` |
| PX4 target / ID | `xplane_qtailsitter` / `5021` |
| Parameters before merge | Draft defaults plus isolated launcher below |
| Parameters after merge | Official PX4 defaults at the merge commit or later |

PX4, plugin mappings and documentation retain the QuadTailsitter name. Only
the owner's local X-Plane aircraft folder/file uses a distinct installation
name; it does not create a new PX4 airframe or alter the physical ACF contents.
Recorded ACF
SHA256 is `c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918`.
Four motors retain their ordering, rotation/cant, CG, inertia and power limits.
The existing four-channel bridge mapping, `airspeedSource=body_axis`,
`pitotAxisBody=-Z` and zero actuator smoothing are unchanged.

The full `config/px4_params/5021_xplane_qtailsitter` reference contains the
pinned inherited defaults plus all 91 reviewed overrides. The last four gain/
trim changes alone do not reproduce the tune. PX4 does not read that reference
from the plugin folder automatically. A stock pre-merge `make ...
xplane_qtailsitter` still uses the old PX4 checkout's defaults.

## Before Merge: Explicit Profile Launch

Keep the normal [official PX4 workflow](https://docs.px4.io/main/en/sim_xplane/)
for the other aircraft. For the owner-supplied compact model only, stop PX4,
load the matching aircraft and select `Plugins > PX4 X-Plane > Airframes >
QuadTailsitter`, then `Connect to SITL`. From the px4xplane checkout in WSL:

```bash
python3 tools/run_compact_qtailsitter.py --px4-path "$HOME/PX4-Autopilot-Me" \
  --xplane-root '/mnt/h/X-Plane 12' --check
python3 tools/run_compact_qtailsitter.py --px4-path "$HOME/PX4-Autopilot-Me" \
  --xplane-root '/mnt/h/X-Plane 12'
```

The runner reuses the existing PX4 checkout/cache, reports the exact source,
checks the model/config/profile, builds SITL and uses its own runtime parameter
directory. It performs no fetch, checkout, reset or cherry-pick and does not
need TruthCapture, an upload token or a diagnostic firmware patch. WSL NAT
host detection can be overridden with `--host` and `--qgc-host`.
The on-disk hash is not proof of X-Plane's loaded in-memory model; verify the
loaded aircraft before connecting. Do not arm until the parameter gate passes.

Saved parameters stay isolated from all other aircraft. If changed parameters
fail the gate, inspect them; `--reset-params` explicitly reapplies the frozen
profile values while retaining other parameters. Do not use PX4-wide
`param reset_all` or delete another aircraft's files.
See `--help` for the source-validation override: a different source must be
reported as unvalidated, not silently accepted as the recorded flight build.

## After Merge

Once the PX4 draft is merged, the normal official target uses the replacement
model's defaults. Confirm the merge commit, install the owner-supplied model,
keep the matching plugin section, and initialize a separate parameter store.
The optional profile runner remains useful for source/profile checks; the
main helper's experimental PR mode is not required for normal operation.
Do not activate the replacement profile with the legacy aircraft. Keep the old
model and old profile paired privately if historical comparison is needed.

## Evidence And Limits

R16 was flown using official PX4 `649929706f` plus a logging-only child
`58794523a1`, px4xplane 4.2.1 and the ACF hash above. The subsequently fetched
upstream `e4aa189ade` has one additional EKF heading-observability change; it
was not the flight-tested binary. The new draft code has not been flown yet.

The matching ULog has no dropouts or in-flight parameter edits; all 91 overrides
match. Complete independent truth contained 72,957 frames. The run completed
mission takeoff, FW, RTL, angle-based back-transition, Land and auto-disarm.
It used a 50 m takeoff target and about 25 s pre-RTL FW, not a complete envelope
test. Normal descent tracked 1.6 m/s; back-transition completed by angle in
13.77 s, with about 37 m climb and a roughly 32 degree recovery pitch peak.
Residual FW roll and speed/height oscillations remain. No hardware readiness,
universal stability or isolated causal A/B result is claimed.

Inherited `FW_R_TC=1.5` exceeds its metadata maximum; changing it to 1.0 changes
outer-loop gain by 50%, so this is an explicit draft blocker, not silent cleanup.
Inherited wing-span/height clearance metadata and unmeasured stall/maximum-speed
limits also need review before production promotion. Do not hide these by
labelling the draft fully tuned. The other four aircraft are unchanged; their
future tuning is separate.

Raw flight/truth archives contain the private ACF and location data. Do not
attach them to public issues/PRs or release assets. Numerical summaries and
reviewer-access arrangements must not accidentally redistribute the model.
