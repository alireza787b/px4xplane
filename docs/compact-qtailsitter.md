# Compact QuadTailsitter Experiment

This developer-only experiment pairs a 2 kg, 0.6 m span, four-motor tailsitter
with model-specific parameters. It is not a retune for the public legacy
5 kg QuadTailsitter example. Both retain the public QuadTailsitter name,
plugin section `[QuadTailsitter]` and target `xplane_qtailsitter`
(`SYS_AUTOSTART=5021`); the target ID alone does not identify the model/tune.

## Profile Pairing

| Use | Parameter reference | Model |
| --- | --- | --- |
| Public example | `config/px4_params/5021_xplane_qtailsitter` | Matching legacy QuadTailsitter |
| Opt-in experiment | `config/experimental/compact-qtailsitter/5021_xplane_qtailsitter` | Matching owner-provided compact model |

The public reference/default pairing remains legacy. The experimental file
holds the complete 211-value compact profile, not just the final gain/trim
changes, and is not shipped as the normal default. PX4 executes airframe
defaults from its own checkout, not from the plugin's reference folder.
Neither plain `px4xplane` nor `--validation` selects this experimental profile.

[PX4 #29598](https://github.com/PX4/PX4-Autopilot/pull/29598) remains **DRAFT**
for future tuning. It is not merged, a release prerequisite or a reason to
replace the public model/default pairing.

## Development Setup

The compact ACF is owner-provided and stays private. Obtain its matching
revision through the owner's installation instructions; there is no public
model download or required drive/path in this guide. Verify the actual loaded
aircraft, mapping, source revision and full parameter profile before arming.
The existing four-channel mapping uses `airspeedSource=body_axis`,
`pitotAxisBody=-Z` and zero actuator smoothing.

`tools/run_compact_qtailsitter.py` remains in developer source and the private
kit, not a public guided-launcher dependency or release asset.
Its use belongs with the matching private model instructions, not the beginner
workflow. Keep experimental saved parameters isolated from public airframes;
inspect differences and back up the selected runtime before an explicit reset.
Do not erase another aircraft's tuning or use a PX4-wide reset as routine setup.

## Evidence and Remaining Work

An earlier owner test completed mission takeoff, fixed-wing flight, RTL,
back-transition, landing and auto-disarm on its recorded build. Residual
fixed-wing oscillations and back-transition climb remain; this is not a
full-envelope, hardware-readiness or new live-flight acceptance claim for PR21.

Review inherited `FW_R_TC=1.5` against its metadata limit before promotion;
silently changing it to 1.0 would change outer-loop gain by 50%. Wing-clearance
metadata, stall/maximum-speed limits and payload/CG behavior also need further
review. Keep any model-specific parameter proposal separate from core-controller
fixes and the public launcher.

Raw aircraft-bearing logs and truth archives remain private. Share sanitized
metrics for public review, not the ACF or access credentials. Truth capture,
uploads and private services are optional investigation tools, not normal-user
dependencies.

Return to the [Developer Guide](DEVELOPER.md) or
[documentation index](index.md#troubleshooting) for general setup issues.
