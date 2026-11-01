# Compact Differential-Thrust Tailsitter: Implementation Review

Revision: 2026-11-01.ref6.5. Experimental SITL, NOT hardware settings.
No perfect-flight, certification or production-readiness claim is made.

## Decision

Keep one experimental aircraft identity, QuadTailsitterRef6, and replace its
managed model in place. Leave the original QuadTailsitter, production plugin,
other airframes and official PX4 unchanged. The existing diagnostic build is
retained; this revision adds no PX4 controller patch or private dive feature.

| Item | Implemented candidate | Basis / limitation |
| --- | --- | --- |
| All-up mass | 2.0 kg | Component budget; not a weighed aircraft |
| Motor reference | Four MAD BSC3120 1000KV, 6S | Manufacturer lists 146 g each |
| Propeller | Two-blade 10x8 geometric surrogate | APC reference, not recovered APC geometry |
| Shaft-power cap | 500 W per motor | Design cap, not established continuous thermal rating |
| RPM ceiling | 13,000 | Separate ceiling, not a forced loaded operating point |
| Projected span | 0.60 m, each crossing pair | Actual serialized mesh extents checked |
| Chord | 0.10 m | All root/tip/rib representations updated |
| Main panel area | 0.13432 m2 | Four swept semipanels; no double-counting |
| Hub coordinates | X-Plane x=+/-0.30, z=+/-0.15, y=-0.20 m | Original motor ordering retained |
| Nominal disk clearance | 46 mm | Static geometry, not flex/tolerance allowance |
| CG | X-Plane (0,-0.245,0) m | 5 mm forward of wing-only quarter-chord estimate |
| Stall / minimum / trim / max | 17 / 21 / 26 / 34 m/s | Screening seeds, not measured polar |
| Blend / transition airspeed | 20 / 25 m/s | Must verify actual wing lift in transition |
| Initial hover parameter | 0.153 | Similarity estimate, not a measured thrust fraction |

The 2 kg budget includes 584 g motors, 450 g battery, 120 g ESCs/wiring,
60 g avionics, 500 g structure and 286 g equipment allowance. Battery placement
solves the CG moment balance. No ballast is added simply to suppress oscillation.
The 450 g / 6S 3 Ah battery is an assumed budget, not a selected product.

## Why These Changes

Hover thrust-to-weight is approximately ONE. A requested 0.4 at liftoff can mean
normalized command, not thrust-to-weight; there is no universal conversion.
The old 6 kg aircraft's measured hover command near 0.71 does not prove that
the compact aircraft needs 0.71. The new seed scales the measured command using
P proportional to T^(3/2), unchanged prop geometry and corrected power units.
Reynolds effects and the simulator's command-to-power law remain uncertainties.

Plane Maker documents propeller-engine power in HORSEPOWER. The previous
generator incorrectly labeled this field kW. The new 500 W cap is serialized as
500/745.699872 = approximately 0.670511 hp, not 0.5. This is a model-generator
unit correction, NOT a newly established PX4 bug. Actual shaft output must still
be checked in truth data; installed altitude behavior is not inferred from units.

The supplied motor chart uses 10x5x3, not 10x8E. Its graph and table also differ
in stated supply voltage (24 versus 25 V). The printed 523 W rated-output entry
must not be confused with the table's similar 50%-command input-power value.
Neither the advertised 5.9 kg maximum nor 105 A peak establishes a continuous
rating for our installation. No such rating is claimed here.

The four inherited secondary aerodynamic panels are disabled rather than
retaining their old trim contribution after shrinking the main wings. Main
mesh, rib chords, body/gear scale, motor hubs, payload CG and power units are
updated together. This creates a new aerodynamic candidate, not a proven trim.

## Reproducible Screening

`build_tools/qtailsitter_sizing.py` generates sizing.json: component mass/CG,
box-plus-parallel-axis inertia, propeller power/RPM/voltage screening and a
mass/CG/density/voltage grid. It evaluates 1.75-2.5 kg, 0-10% chord CG offsets,
densities 1.0/1.059/1.225 kg/m3 and 19.8/22.2/25.2 V.

At 2 kg and density 1.059, the static BEM surrogate with a 500 W cap predicts
about 3.40 total thrust/weight AFTER a 20% thrust derating. CLmax=1.0-1.2 gives
stall estimates 16.61-15.16 m/s. These are screening calculations, not measured
flight envelope limits. The derating is not a statistical confidence bound.

The wing-only CG offset predicts about 0.098 N m trim demand at 1g. Actual body,
propwash, thrust-line offsets and Cm-alpha can change both trim and stability.
The model does not establish an optimum CG. Any future CG change requires the
motor coordinates relative to CG and mass/inertia budget to be regenerated.

Component inertia in X-Plane axes is approximately diagonal
(0.02090, 0.08569, 0.07210) kg m2. This is NOT the simulator's measured tensor.
Automatic X-Plane inertia is retained: undocumented manual-inertia units are
not guessed. Compare runtime response before accepting the derived gain model.

## Gain Method And Limits

MC rate P seeds use I*desired_bandwidth / differential-moment slope, with
4/4/2 rad/s trial bandwidths. The moment slope uses the local P~T^(3/2)
derivative and PX4's normalized rectangular mixer, not bare motor arm lengths.
PX4 shares roll/pitch normalization; a compiled test of its recorded matrix,
filter and normalization routines verifies the resulting slopes and torque axes.

Small integral/derivative seeds and zero unidentified feedforward are explicit
trial choices, NOT identified optimal gains. For the pitch axis, PX4's global K
preserves the calculated effective gain while keeping displayed P at its 0.01
documented minimum. FW roll/pitch seeds additionally account for differential
thrust scaling. No inherited feedforward or derivative term is silently retained.

| Axis | P | I | D | K |
| --- | ---: | ---: | ---: | ---: |
| MC roll | 0.031815 | 0.007954 | 0.000636 | 1 |
| MC pitch | 0.010000 | 0.002500 | 0.000200 | 0.922037 |
| MC yaw | 0.105262 | 0.020000 | 0.002105 | 1 |
| FW roll | 0.287079 | 0.020000 | 0 | n/a |
| FW pitch | 0.007684 | 0.001921 | 0 | n/a |

The earlier allocation repair remains: CT=6.5, rather than CT=2 whose small
reaction-torque row fell below the recorded allocator's 0.05 weak-axis filter.
This is a configuration repair. It does not establish an upstream PX4 defect.
No EKF bias limits, sensor gains or health checks are relaxed by this revision.

## Workflow And Verification

The updater requires stopped PX4 and, for an installed ACF change, stopped
X-Plane. It verifies bundle and installed hashes, replaces the SAME ACF and
receipt, journals managed and installed bytes, and rolls back interrupted writes.
Meaningful local aircraft edits are refused, not discarded. Credentials and
flight runs are preserved. One previous managed version is retained.

Automatic current-run truth snapshot, archive and authenticated upload after
shutdown remain enabled when privately configured. Failures retain local ZIPs.
The runner does not stop the recorder, auto-arm, or modify production installs.

Host tests cover geometry representations, original-source preservation, power
units, component budgets, rotor positions, allocator normalization, failed
updates/rollback, truth snapshots and upload handling. Tests cannot establish
X-Plane stability. No simulator flight or human maintainer review occurred here;
independent agent review was unavailable because its authentication failed.

FIRST run: `bash run.sh hover --yes-debug`. Use commander takeoff, observe hover,
commander land and shutdown. No transition on this new model's first run.
Review actual mass, RPM, shaft power, motor saturation, estimator consistency,
rate tracking and descent. Proceed to full flight only after that evidence passes.
Do not force arm, increase EKF limits or use this SITL profile on hardware.

## Primary References

- [MAD BSC3120 product and specification chart](https://webmail.mad-motor.com/products/bsc-3120-fpv-drone-motor)
- [APC 10x8E analytical performance](https://www.apcprop.com/files/PER3_10x8E.dat)
- [APC RPM limits](https://www.apcprop.com/technical-information/rpm-limits/)
- [Plane Maker engine power and geometry](https://developer.x-plane.com/manuals/planemaker/)
- [PX4 MC PID tuning guide](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html)
- PX4 code pinned at 9cfe130a8d8e0c25f88e044141c523247ca53d48:
  ControlAllocationPseudoInverse.cpp and mc_rate_control_params.yaml.

PROPULSION_REVIEW.md contains the earlier 6 kg propeller-method review; its
old vehicle numbers are not the current specification. parameters.json records
the reason for each current parameter and design.json records ACF before/after.
