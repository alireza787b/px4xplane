# R13: Current Upstream Recovery And FW Test

Experimental SITL only. No production-readiness or hardware claim. R13
supersedes the old R12 test recommendation. No R13 X-Plane flight has yet been
observed here.

## Important Correction

The R12 investigation correctly measured repeated braking and retained
acceleration, but missed a relevant fix already upstream: [PR #28350](https://github.com/PX4/PX4-Autopilot/pull/28350),
merged 2026-10-12 as `2750a9bd274476443548d8ed3efdfbd6fa8fd5b4`.
It explicitly addresses acceleration retained on release, velocity reseeding
and controller-limit clipping. We should test that implementation, not create
a duplicate patch or continue treating the old base as current PX4.

The old official base `f90b40d61e56405513dfcd8e194287ac60d3a6ae` is dated
September 21 in its original timezone (2026-09-22 04:57:37 UTC). Its later
diagnostic-only commit did not advance that base. It included #28170, not
#28350. Earlier R12 claims about preserving per-cycle velocity reseeding are
superseded by the final upstream implementation.

## Exact Firmware

| Item | Identity |
| --- | --- |
| Official main snapshot, observed 2026-11-03 | `649929706fb34bf0b06dbd2df24e04d5d452d616` |
| Official snapshot commit time | 2026-11-02 23:10:05 UTC |
| Same official snapshot on our fork | `validation/official-upstream-20261103` |
| Official plus log-only observations | `58794523a151fa33a28a564454454b8c9fbfcdd5` |
| Diagnostic fork branch | `validation/qtailsitter-upstream-20261103` |
| Runner revision | `2026-11-03.ref6.13` |
| Unchanged ACF SHA256 | `c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918` |

These are reproducible development-main pins, not a stable PX4 release.
The fork's existing PR branches and main user checkout are not reset or
force-pushed. The diagnostic child changes only logging in five files;
all control behavior comes from official upstream. No private dive option or
unmerged GSF/TECS/VTOL patch is added.

The bundle carries official Git history newer than the old September base, so
an existing old PX4 checkout can import it without cloning PX4 again. Missing
submodule objects still require downloads. Before compilation, submodules are
aligned to the pinned commits and their identities are recorded. Source and
previously initialized submodule identities are restored after shutdown or
failure; dirty user work is refused, never reset.
Clean submodule worktrees first initialized by this test are deinitialized on
restore, while their Git object caches are retained. This prevents a newly
introduced upstream module leaving the older user branch dirty. Edits made
during the run stop restoration rather than being forced away.

## What Was Seen In R11

Original archive SHA256:
`bcbd3657db08a5382a209b08fa3aa85044ae6e868004307bd54fefe279e51814`.
ULog `2026-11-02/19_15_01.ulg`; complete 97,184-row independent truth capture.

- Initial settled hover was quiet: horizontal speed RMS 0.0148 m/s and attitude
  error RMS 0.140 degrees in the reviewed window.
- Front transition began at boot 68.008 s; FW at 80.711 s.
- FW roll/body-MC-yaw oscillated near 0.448 Hz, before back-transition.
- Back began at 109.540 s and exited at 124.438 s by the actual pitch criterion,
  at about -14.43 degrees; horizontal speed was still about 9.3 m/s.
- Braking released with considerable trajectory acceleration retained.
  Pre-edit releases averaged about 4.83 m/s2 horizontally. Normal jerk was 0.7.
  The virtual trajectory repeatedly exceeded the 5.2 m/s trigger.
- Land at 162.452 s was accepted. Its raw down-speed request stayed 1.4 m/s,
  while executed descent averaged about 0.19 m/s as braking repeatedly resumed.
  This was not a rejected Land command due to lack of stationary flight.
- First manual gain change was not until 221.032 s. The original failure
  preceded the slider experiments and later inappropriate FW autotune in MC.
- Around 795-800 s power fell as commands approached saturation. This separate
  event is strongly consistent with battery depletion; it does not explain the
  early recovery failure. R13 records battery state directly.

The old pinned smoother reproduced overspeed after all 25 recorded releases
at jerk 0.7, and none at jerk 4. This was a smoother-only screen, not an aircraft
simulation. It did not include the newer upstream release reset.

## What The Upstream Fix Actually Does

Read the [merged code](https://github.com/PX4/PX4-Autopilot/commit/2750a9bd274476443548d8ed3efdfbd6fa8fd5b4),
not only the PR body, which describes an older shared-flag design.

- Checks measured as well as trajectory velocity for activation.
- Seeds trajectory velocity from measurement once on activation, not every cycle.
- Keeps position reseeding while braking.
- Uses measured speed for release and retains a no-progress watchdog.
- Clears trajectory acceleration on release so ordinary guidance does not
  inherit the high braking-acceleration state.
- Temporarily extends controller velocity bounds after controlled-axis changes:
  measured entry-speed seed, one-second hold, then 5 m/s2 decay to normal bounds.
- Applies the takeoff ramp after the temporary bounds so it is not overridden.

This directly addresses the observed mechanism. It does not prove that every
back-transition, rate loop or aircraft model is now correct. There is no new
independent braking-threshold parameter in this test.

## Parameter Changes

R13 retains the same ACF, mass/CG, motor directions, allocation, MC/FW rate PID,
attitude gains, NPFG settings, airspeed envelope, trim and transition duration.
Only these generated trajectory-profile values change:

| Parameter | Inherited R11/R12 baseline | R13 | Reason |
| --- | ---: | ---: | --- |
| MPC_JERK_AUTO | 0.7 | 4 | Current PX4 default; old value below documented minimum 1. |
| MPC_ACC_HOR | 0.7 | 2 | Current documented minimum; less restrictive nominal recovery acceleration. |
| MPC_ACC_HOR_MAX | 1 | 2 | Current documented minimum for manual direct-velocity deceleration; unused with MPC_POS_MODE=4. Not an auto emergency-brake setting. |

Cruise stays 3.5 m/s and horizontal maximum stays 4 m/s. No higher braking
threshold is used to hide the problem. More acceleration is not a guaranteed
stability improvement: this is an explicitly combined firmware/profile trial,
not an isolated A/B attribution to #28350. It must be evaluated in the new log.
[Trajectory definitions](https://github.com/PX4/PX4-Autopilot/blob/649929706fb34bf0b06dbd2df24e04d5d452d616/src/modules/mc_pos_control/multicopter_autonomous_params.yaml).

## FW, TECS And Flight Quality

The latest official snapshot includes relevant changes absent from the old base:

| Upstream change | Expected relevance, not a flight result |
| --- | --- |
| [#29009](https://github.com/PX4/PX4-Autopilot/pull/29009), `b4134d75f2054943c23ad34a8bb636c5ec7e8509` | TECS avoids trading altitude for unattainable speed above trim under normal speed weighting. |
| [#26859](https://github.com/PX4/PX4-Autopilot/pull/26859), `fe0662edb1e7b71b966b4c972280a206e8ceee5f` | Earlier underspeed protection, altitude-reference handling and bank limiting during recovery. |
| `2395526999`, true-airspeed turn coordination | Corrects the speed used for coordinated-turn feedforward. |
| TECS controller reset/filter/integrator corrections | Included as official upstream, not cherry-picked duplicates. |

The R11 evidence does not identify an optimum new FW PID or prove a cant,
inertia or thrust-limit correction. Increasing FW roll P again without the
missing moment measurements would repeat the prior unsuccessful method.
We keep the gains unchanged and record total/prop/aerodynamic body moments,
forces, runtime inertia and electrical state.

An inherited FW_R_TC=1.5 is above current metadata maximum 1.0. It is retained
explicitly for this first upstream comparison rather than silently increasing
outer-loop bandwidth. A later attitude-time-constant trial must follow inner
rate-response review. Speeds/stall/trim are still provisional and need measured
steady flight; no 26 m/s trim or power calibration is newly claimed here.

## Historical Research: What Was Decided

- [Issue #18440](https://github.com/PX4/PX4-Autopilot/issues/18440) reported
  high-speed recovery on real quadplane hardware and SITL, not X-Plane alone.
  Maintainers favored controlled emergency stopping rather than abrupt clipping.
- [#18614](https://github.com/PX4/PX4-Autopilot/pull/18614) was closed unmerged
  because it was replaced by [#18740](https://github.com/PX4/PX4-Autopilot/pull/18740),
  merged 2021-11-30. Earlier omission of this successor is corrected.
- [#18805](https://github.com/PX4/PX4-Autopilot/pull/18805) merged 2021-12-08:
  use the maximum horizontal speed rather than cruise for the trigger.
- [#28170](https://github.com/PX4/PX4-Autopilot/pull/28170) merged 2026-08-21:
  prevent braking velocity-error reversal through reseeding; avoid noise-latched
  stopping with a 1 m/s release tolerance.
- [#28350](https://github.com/PX4/PX4-Autopilot/pull/28350) merged 2026-10-12:
  follow-up fixes the release state and controller-limit interaction.
- [This tailsitter forum thread](https://discuss.px4.io/t/first-flight-tailsitter-unstable-hover-video-and-logs/29587)
  suggested pitch-rate gain reduction for saturation, but ends without a proven
  tune and includes elevons. It is not proof of the same failure or our solution.

No exact public reproduction of our complete rotor-only FW/back/Land sequence
was established. The merged release-acceleration fix is a close code-level
match; the earlier forum oscillation is only a symptom-level resemblance.

## Next Flight And Verification

Follow QUICKSTART.md. One short, fully charged, no-wind flight: console takeoff
to actual 180 m, settled hover, front-transition, 20 seconds established FW,
normal back-transition, settled MC, Land, touchdown/disarm, shutdown.

No GoTo/RTL, autotune, speed commands or in-flight gain changes. Stop a sustained
failed recovery after 15-20 seconds rather than leaving it airborne for minutes.
Keep total run within six minutes. Missing new truth fields stop a full-flight
test before source changes; no measurements are synthesized.

Host tests, parameter-name checks, package/update validation and the local
build result are reported in the delivery handoff. They do not certify an
X-Plane flight. If either FW or MC still oscillates, use measured rate/attitude,
moment balance, saturation, braking state and raw/executed descent to select
the next change. No new public PR, production plugin release or hardware image
is created for this experimental iteration.

The exact diagnostic SITL firmware compiled locally, and a no-simulator PX4
execution accepted and compared all 82 generated profile values. This did not
arm a vehicle or run an X-Plane flight. The resource-constrained local build
skipped the all-platform submodule sweep after aligning its required SITL
dependencies; the delivered runner uses normal checks and exact submodule
alignment. The five-file diagnostic diff was reviewed against official base;
braking activation/release logic, controller, FW rate/attitude and TECS source
are checked unchanged by host regression tests.
