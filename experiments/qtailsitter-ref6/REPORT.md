# R11: R10 Flight Review And Next Combined Tuning Trial

Experimental SITL only. No hardware or production readiness claim.
Same aircraft, physical axes, motor rotation, plugin and diagnostic PX4 pin.
Full flight has not been demonstrated on this server.

## Evidence And Outcome

Manually supplied ZIP SHA256:
09339ba4e92076c8db814408f99f788e546612b786a90419433b56fd5e903ae2.
ULog: 2026-11-02/16_20_42.ulg. Runner2026-11-02.ref6.10.
PX4 diagnostic9cfe130a8d8e0c25f88e044141c523247ca53d48,
official basef90b40d61e56405513dfcd8e194287ac60d3a6ae.
Loaded-path ACF hashc70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918.

The intended R10 gain was applied: FW_RR_P=0.10, MC_ROLLRATE_P=0.060.
No in-flight parameter edits or ULog dropouts. Initial ULog parameters precede
the startup overlay; effective values include changed_parameters before arming.

| Event, PX4 boot time | Seconds |
| --- | ---: |
| Front transition | 70.934 |
| FW established | 84.290 |
| Back-transition | 110.058 |
| MC label | 125.068 |
| Land requested | 336.523 |
| Touchdown / automatic disarm | 683.840 / 685.342 |

R10 eventually landed; it did not complete a clean recovery/descent.
QGC requested approximately57 m above local origin, not the console180 m
scenario. FW exposure was25.77 s rather than15 s. These differences are recorded,
not used to dismiss the observed control problems.

## FW Improved, But Remains Oscillatory

Correct tailsitter conversion: physical MC yaw is negative FW roll.

| Established FW metric | R8 | R10 |
| --- | ---: | ---: |
| FW roll-rate error RMS rad/s | 0.378 | 0.256 |
| Actual FW roll-rate RMS rad/s | 0.321 | 0.209 |
| FW bank RMS deg | 14.83 | 9.76 |
| Approximate dominant rate frequency Hz | 0.44 | 0.44 |

R8 and R10 were different routes/windows, not isolated identical A/B trials.
These measurements support continuing the feedback experiment, not a causal
proof or an identified optimum. R10 still banked about-10.2 to+24.3 degrees.

For R11, FW_RR_P becomes0.14. The downstream differential scale stays0.55,
so effective physical-axis P becomes0.077. FW I/D/FF, outer time constants,
yaw/pitch gains and differential scales stay unchanged. Earlier prop-response
evidence did not show a sign reversal or near180-degree actuation lag.
Do not simply classify every visible oscillation as excessive rate P.

## Why Land Descended In Bursts

This is not explained by initiating descent at low power:

- At302.11 s braking briefly released, then reactivated at303.87 s.
- Roll exceeded10 degrees at315.89 s, BEFORE Land at336.52 s.
- Between336.6 and590 s, raw Land descent was1.40 m/s, but downstream demand
  averaged0.151 m/s, actual descent0.155, and logged groundtruth0.153.
- Collective thrust command was0.238-0.308; motor commands0.228-0.335.
  No allocator torque/thrust failures were logged in the reviewed Land interval.
- There were85 total braking activations,76 during Land, all Land activations
  horizontal-triggered. Braking occupied about61% of Land diagnostic samples.

Braking requests all-axis stopping and repeatedly interrupts descent while
lateral motion grows. Raising descent speed or reducing aircraft mass would
not address that chain. The brief thrust minimum near125.53 s is a separate
back-transition transient, not evidence that low collective causes the
much later landing oscillation.

The time-ordering rules out Land initiation as the original trigger. It does
not prove that every aspect of braking/guidance is ideal or that allocation
residual zero means physical force accuracy.

## What Needs Tuning In MC

The Land cycle is approximately6-7 s. In two reviewed windows:

| Tracking phase lag | 336.6-450 s | 450-590 s |
| --- | ---: | ---: |
| Roll body-rate response | 12.8 deg | 10.2 deg |
| Roll attitude response | 56.7 deg | 49.0 deg |
| Pitch attitude response | 75.5 deg | 70.0 deg |

The inner rates follow relatively closely at this frequency, but the requested
attitude is reached too late. Increasing rate P again is not the best-supported
first action. Strengthen the outer attitude response:

| Parameter | R10 | R11 |
| --- | ---: | ---: |
| MC_ROLL_P | 0.90 | 1.40 |
| MC_PITCH_P | 0.58 | 1.00 |
| FW_RR_P | 0.10 | 0.14 |

All MC rate PID, FW I/D/FF, yaw gains/weight, position/velocity PID, tilt limits, thrust limits
and speed/acceleration/jerk settings remain unchanged. These three changes make
R11 a COMBINED experiment. It will not isolate one universally optimal gain.

Local scalar-model screening predicts roll settling3.25->1.27 s and
pitch5.97->3.22 s for small ideal angle steps. Model sensitivity includes roll
overshoot up to roughly11%. These are descriptive closed-loop model screens,
not coupled simulator flight predictions, stability margins or a guarantee.
The MC attitude changes also affect BOTH transitions; they cannot be treated
as pure-MC-only changes. Judge front, FW, back, recovery and Land separately.

Independent Astra review supports this MC candidate and its limitations.
This is agent review, not human PX4 maintainer approval.

## Back-Transition Is Not Yet A Pass

R10 still exited through its15 s timer, with pitch approximately-59.3 degrees,
not the approximately-14.9 degree angle criterion. It then recovered upright
later under ordinary MC guidance. Visual improvement and eventual landing
are not proof of a good transition handoff.

The larger pitch attitude gain may improve tracking, but aerodynamic moments
and thrust/inflow changes still require evidence. Do not lengthen the timer,
raise a throttle floor or reverse actuators to conceal this failure.
R11 preserves transition timing and measures actual exit condition again.

## Weight, CG, Power And Sensors

No ACF redesign is justified by this recording. Motor command headroom and lack
of logged saturation do not prove perfect physical authority, but do not support
adding ballast, changing CG or increasing power as the next fix.

The independent CSV truth window is MISSING: the runner rejected it above
256 MiB. Therefore actual prop thrust/RPM/power and independent mass/wind
cannot be revalidated for this flight. Logged groundtruth shares the plugin
transform, so it is not an independent force-model check.

Attitude disagreement with logged groundtruth is0.526 degrees RMS, rate
disagreement about0.008 rad/s per axis, and accelerometer-bias RMS approximately
0.0108/0.0069/0.0102 m/s2. No repeated estimator fault explains the symptoms.
No EKF limit or production sensor-pipeline change is warranted here.

## Speeds And TECS

R10 CAS was25.675-27.888 m/s, median26.494, around the26 m/s trim target.
TECS true-airspeed target wasabout28.054 m/s: TAS and CAS differ with density,
so that target is not evidence that cruise trim was silently raised.
Throttle command0.109-0.254 and stable FW height request do not show max-power
cruise starvation in this interval.

Keep TECS/speed settings until attitude tracking is established, consistent
with the PX4 tuning sequence. MC limits remain cruise3.5/max4 m/s, automatic
climb2/descent1.4 m/s, near-ground Land0.9 m/s. The observed slow descent was
mostly inhibited command, not a demand for more speed. Faster manoeuvres are
a later envelope test, not a substitute for damping.

## Evidence Workflow Fixes

1. Complete truth windows above256 MiB now stream directly to frames.csv.gz.
   No raw duplicate or frame decimation. Input ceiling1 GiB, compressed ceiling
   128 MiB, resource checks, complete-row parsing and raw/stored hashes remain.
   Atomic failures leave source captures untouched. Analysis reads CSV or gzip.
2. The original upload TLS EOF reproduced from the server too. Free space was
   only379 MiB versus the intake384 MiB reserve BEFORE receiving the~57 MiB ZIP.
   After freeing96 MiB of archived journals, the SAME upload succeeded with
   checksum/byte receipt. A further188 MiB npm download cache was removed.
3. Authenticated HEAD admission now reports quota/free-space rejection before
   sending the large body. POST rechecks bounds; no reservation or lowered
   security/storage limit. TLS/network failures remain possible and retryable.
   Credentials did not change; no need for token reconfiguration.

These are DEBUG-runner/server changes, not PX4-core or production-plugin edits.
They cannot recreate truth omitted from an already submitted ZIP.
Do not delete the original Windows recording if later force analysis is needed.

## Next Test

Use QUICKSTART.md / PX4_REF6_CURRENT_TEST.md and the SAME installed debug folder.
Update, launch flight, use CONSOLE takeoff to180 m, settle, front transition,
15 s established FW, normal back-transition, actual settled MC, then Land at
current position and shutdown. No RTL, GoTo, gain edits or speed changes.

If recovery/landing remains uncontrolled, capture10-15 s then shutdown rather
than waiting minutes. If it is controlled, let descent/touchdown/disarm complete.
Truth snapshots and uploads remain automatic. No new ACF installation required.

Acceptance targets: lower FW roll oscillation without worse coupling/clipping;
back exit by actual pitch criterion; prompt MC damping; unbroken descent
request/response without repeated horizontal braking. Not a production-demo
or hardware authorization.

## Reproduction

- tools/analyze_qtailsitter_handoff.py ZIP NEW_DIR now explicitly handles absent
  truth: ULog analysis is retained but force/independent-CSV claims are withheld.
- evidence/mc_resume.py and review.py reproduce cached MC metrics from the
  original R10 ZIP; use RAM for recreated data. No large arrays ship in bundle.
- Host tests cover complete >256 MiB compressed capture, reset/rewrite/limits,
  collector compatibility, gzip analysis, parameter identities and upload bounds.

[PX4 MC tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter):
rate loop first, then outer response.
[PX4 FW tuning](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing):
rate feedback and attitude time constants.
[TECS preconditions](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing):
establish attitude control before energy-controller tuning.
