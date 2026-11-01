# R6: Short Test A After R5 Hover Failure

Experimental SITL only. Revision 2026-11-01.ref6.6. No PX4 core change,
private dive, production plugin change or hardware tune is included.

## What The Uploaded Evidence Shows

Archive SHA256: 24df8b40f2a6f303c96b3ffe470e8fb64acae2162fa61a6a4031c497347aefaa.
Recorded runner r5, diagnostic source 9cfe130a8d8e0c25f88e044141c523247ca53d48.
It stayed in MC throughout; this was not a failed back-transition.

| Observation | Evidence |
| --- | --- |
| Actual mass was not 2 kg | Truth m_total median 2.250000715 kg, fixed payload 0.250000179 kg |
| Liftoff was delayed | Takeoff request 40.041 s; EKF height above 0.5 m at 61.325 s: 21.284 s later |
| Hover value was too low | r5 seed 0.153; HTE median 0.298227 at boot 65-85 s |
| Pitch response became unstable | Increasing pitch-rate/attitude error and horizontal departure before crash; see hover.svg |
| A reversal is not established | Pre-divergence pitch acceleration has positive correlation with requested pitch torque in selected windows |
| Generator changed directions | Original engine psi +5,+5,-5,-5 degrees was overwritten with zero in r4/r5 |
| Channel order / rotation were preserved | Channel i remained engine i; prop signs remained +,+,-,- |

Closed-loop correlation is not a proven plant identification or proof that all
axes are correct. It supports investigating response authority rather than
blindly inverting signs. Original orientations are restored independently of
that inference: the performance redesign did not require changing them.

## Implemented Changes

1. Preserve original ACF engine and blade orientation fields and propeller
   rotation signs verbatim. Motor quadrant order and channel mapping stay
   unchanged. Compact hub distances remain +/-0.30 m and +/-0.15 m.
2. Restore the existing official 5021 rotor direction vectors, not a newly
   invented Euler conversion. CT=6.5 remains; positions follow the compact ACF.
   Physical sign/force validation remains pending, not claimed completed.
3. Set fixed-payload capacity to zero for this comparison. X-Plane's previous
   half-capacity default silently added 250 g. Empty mass stays 2 kg.
   Runtime mass must still be checked in the next truth capture.
4. MPC_THR_HOVER=0.25: measured HTE 0.298227 mass-corrected by
   (2/2.250000715)^(3/2). Propeller and power cap are unchanged. This is an
   evidence-based starting estimate, NOT a guaranteed hover equilibrium.
5. Trial MC pitch effective P rises from 0.00922 to 0.06, K=1,
   I=0.015, D=0.0012. Selected pre-divergence closed-loop acceleration/torque
   regressions suggest approximately 40-84 rad/s2 per normalized output, giving
   nominal rate bandwidth around 2.4-5 rad/s rather than below 1 rad/s.
   Confounding feedback, delay, restored cant and aerodynamic moments limit
   this estimate; it is not validated optimal PID. FW tuning is not declared fixed.
6. Hold r5 MC roll and FW gains fixed. Recomputing them from the revised hover
   seed would silently increase gains on unrelated axes. Restored modeled yaw
   lever is about 2.54 times reaction torque alone, so r5 yaw P/I/D are reduced
   by the inverse lever ratio. This compensates the modeled cant contribution;
   physical response remains to be measured. Independent review caught this
   previously missing cant term before packaging.
7. `bash run.sh A` is a 10 m hover/Land simulation. No redundant typed-consent
   prompt and no auto-arm. Bench details stay in JSON; one NOT VERIFIED line
   replaces the misleading multi-line PENDING/FAIL display without measurements.

Compact geometry, CG candidate, surrogate propeller and 500 W shaft cap remain.
No extra aircraft or folders. Authentication, identity checks, original-config
backup and automatic upload remain: removing those risks wrong-file tests or
unauthenticated uploads, not better control. The configured token is reused.

## Motor Peak Thrust Is Propeller-Specific

The [MAD BSC3120 1000KV chart](https://webmail.mad-motor.com/products/bsc-3120-fpv-drone-motor)
shows 5.889 kgf at 25 V, 105.44 A, 2636 W input and 15,785 rpm, with a
10x5x3 propeller. The graph header states 24 V while its table states 25 V.
This is not a 500 W or 10x8 two-blade datum. It establishes neither a continuous
thermal rating nor a safe burst duration for our installation.

Our 500 W/motor shaft cap is deliberately derated for this comparison, not a
faithful model of the motor's full burst capability. Do not claim it reproduces
the manufacturer's peak. The [APC 10x8E reference](https://www.apcprop.com/files/PER3_10x8E.dat)
predicts 30.923 N (about 3.15 kgf) at 13,000 rpm and 734.98 W static shaft power;
already above our 500 W cap. A motor maximum does not override propeller RPM
limits or determine thrust without its matching load point. Shaft power,
thrust and RPM are not inflated to force a 5 kgf result.

[Plane Maker](https://developer.x-plane.com/manuals/planemaker/) documents engine
power in horsepower: 500 W is serialized as 500/745.699872 hp. This correction
remains. Prop geometry, automatic simulator inertia and wing-only CG assumptions
still require measurement. No hardware suitability or perfect-flight claim.

## Verification And Next Evidence

Host tests cover orientation/rotation preservation, geometry, payload,
profile consistency, allocator math, update rollback, capture and upload.
A compiled test of pinned PX4 routines checks the restored canted
torque/thrust-Z subset has full rank. Rank is not physical sign validation.
No actual X-Plane flight occurs on this server.

Independent code/evidence review found no reason to reorder motors or invert
pitch. It also noted that identical serialized engine angles for motors 0/1 do
not by themselves prove the opposite official horizontal axis vectors physically
match: the X-Plane interpretation must still be measured. We restore the existing
baseline convention, not claim that this correspondence is newly verified.

Reproduce the small evidence review (numpy/pyulog required):

```bash
PYTHONPATH=/tmp/qtune-review-20261101 python3 tools/analyze_qtailsitter_hover.py LOG.zip NEW_OUTPUT_FOLDER
```

It streams truth and writes metrics.json plus hover.svg, without extracting
the whole archive or assuming truth/PX4 clocks align.

Next: no wind, reload the same updated Ref6, start truth recording, Connect to
SITL, `bash run.sh A`, commander takeoff. Observe 20 seconds only if controlled;
commander land, disarm, shutdown. No transition. Upload retains inputs, source,
ULog, console and this run's truth window. Stop the simulation on growing
departure and preserve evidence; do not force arm or relax EKF limits.

Original aircraft, official PX4 and other airframes remain untouched. Review
the new hover response before scheduling FW flight. Measured evidence takes
precedence over the previous unvalidated model-based expectations.
