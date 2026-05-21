# Report v39 - QuadTailsitter qtail5 Hover Authority Recovery

Date: 2026-05-21

## Scope

This report reviews `/home/alireza/qtail5.zip` and defines the next
QuadTailsitter hover-recovery package. qtail5 used the intended v3.4.26
airframe and is therefore a valid test of the yaw-allocation fix.

## Evidence From qtail5

- px4xplane loaded `v3.4.26` with `Config Name: QuadTailsitter`.
- PX4 loaded `SYS_AUTOSTART=5021` and the expected qtail v3.4.26 defaults:
  `CA_ROTOR*_CT=6.5`, `KM=+/-0.05`, `MPC_THR_HOVER=0.25`, and
  `MIS_TAKEOFF_ALT=1.5`.
- TruthCapture was healthy: `10,263` frames, zero dropped rows, no sim-time
  resets, about `77 Hz` effective callback rate, and max frame period about
  `40 ms`.
- PX4 detected takeoff at `17.77 s`. Pitch attitude failure appeared at
  `43.62 s`, followed by roll attitude failure at `54.27 s`.
- The current ACF is still a `2.27 kg` aircraft. At near full throttle,
  TruthCapture showed about `43-47 N` total thrust. That is enough for the
  current ACF, but not enough for the requested future `6-6.5 kg` physical
  design.

## Findings

qtail5 fixed the qtail4 yaw-allocation fault: control allocation no longer
reported persistent unallocated yaw torque, and yaw motor differential was
visible.

The remaining crash is a hover authority/tuning mismatch:

- PX4 commanded moderate attitude setpoints, then hit pitch-rate recovery
  limits as the aircraft started translating like a tailsitter in forward
  flight.
- `control_allocator_status` reported torque achieved and no sustained motor
  saturation through the early failure window.
- Motor differential was too small for the observed attitude error. Around
  `40-44 s`, pitch setpoint error exceeded `30-90 deg`, but pitch motor
  differential was still only a few percent before the aircraft was already in
  a high-speed tilted state.
- This is the signature of an overestimated control-effectiveness matrix and an
  intentionally softened rate tune, not a sensor dropout or frame-rate problem.

The v3.4.26 `CT=6.5` change was correct for avoiding the yaw-row cutoff, but it
overstated the effectiveness of this X-Plane model. PX4 therefore believed it
had applied enough torque when the actual aircraft response was still weak.

## Implemented Fix

- Changed QuadTailsitter `CA_ROTOR*_CT` from `6.5` to `2.0`. With `KM=0.05`,
  yaw effectiveness is `0.10`, safely above PX4's `0.05` weak-row cutoff, while
  pitch/roll motor differential is materially stronger than qtail5.
- Increased pitch/roll rate authority from the very soft hover-recovery tune:
  `MC_ROLLRATE_K=0.45`, `MC_PITCHRATE_K=0.60`.
- Reduced yaw demand to avoid over-correcting now that `CT` is lower:
  `MC_YAW_P=0.35`, `MC_YAW_WEIGHT=0.25`, `MC_YAWRATE_P=0.04`,
  `MC_YAWRATE_I=0.008`, `MC_YAWRATE_MAX=30`.
- Reduced horizontal/tilt aggressiveness during hover recovery:
  `MPC_TILTMAX_AIR=10`, `MPC_MAN_TILT_MAX=10`, `MPC_TILTMAX_LND=8`,
  `MPC_XY_CRUISE=1.5`, `MPC_XY_VEL_MAX=2`, `MPC_ACC_HOR=0.8`,
  `MPC_ACC_HOR_MAX=1.2`, `MPC_JERK_AUTO=1.0`, `MPC_JERK_MAX=2.0`.
- Raised nominal hover thrust to `0.27` and shortened takeoff ramp to `2.5 s`
  to reduce the near-ground pause seen by the user.

## Physical Model Decision

Do not rescale the ACF to `6-6.5 kg` in this slice. The logged X-Plane thrust
is not sufficient for that mass. A proper physical redesign needs a separate
static-thrust slice: update the motor/prop/battery model, verify hover thrust
margin in TruthCapture, then apply the canted-motor geometry and matching PX4
control-allocation axes.

## Next Test

Use v3.4.27 for one hover-only test:

1. Pull or copy the updated `5021_xplane_qtailsitter` into PX4 and run
   `make distclean` once.
2. Install the v3.4.27 px4xplane package and the packaged QuadTailsitter
   aircraft.
3. Take off and hold. If QGC commands a higher takeoff altitude than `1.5 m`,
   that is acceptable for this test, but do not command transition.
4. After `20-30 s` of stable hover, apply only small manual or Hold reposition
   inputs.
5. Land and wait `10-15 s` after disarm before stopping PX4.

The log should confirm stronger pitch/roll motor differential, no persistent
allocator unallocated torque, no sustained motor saturation, and no attitude
failure.
