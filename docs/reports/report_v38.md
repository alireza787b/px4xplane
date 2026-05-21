# Report v38 - QuadTailsitter qtail4 Yaw Allocation Recovery

Date: 2026-05-21

## Scope

This report reviews `/home/alireza/qtail4.zip` and defines the next
QuadTailsitter hover-only recovery package. The goal is to fix the proven
control-allocation fault without mixing in a broad aircraft mass, propeller, or
canted-motor redesign.

## Evidence From qtail4

- px4xplane loaded `v3.4.25` with `Config Name: QuadTailsitter` and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- TruthCapture was healthy: `11,609` frames, zero dropped rows, no sim-time
  resets, about `77 Hz` effective callback rate, and max frame period about
  `50 ms`.
- The X-Plane model mass was not the requested `6-6.5 kg` test article.
  TruthCapture reported `acf_m_empty=2.2679 kg`, meaning the current
  `_m_empty=5.0` ACF value is being interpreted as `5 lb`.
- The aircraft became barely airborne, then yaw drift grew while the hover
  motors stayed far from saturation. Pitch divergence and attitude failsafe
  followed the yaw loss.
- PX4 logged attitude failures, but estimator timing and TruthCapture timing
  were not the primary trigger.

## Root Cause

The ULog showed PX4 requesting yaw correction while the motor outputs preserved
almost exactly zero yaw differential:

- `actuator_motors` yaw mix term `m0 + m1 - m2 - m3` stayed near zero.
- `control_allocator_status` reported yaw torque as unallocated through the
  failure window while roll and pitch remained allocated.
- The active airframe used `CA_ROTOR*_CT=1.0` with `CA_ROTOR*_KM=+/-0.05`.

PX4's control allocator treats rows with no entry above `0.05` effectiveness as
weak and zeros them. With `CT=1.0` and `KM=0.05`, the yaw row lands exactly on
that cutoff, so yaw can be mathematically removed. This matches qtail4: PX4
asked for yaw torque, but the motors never received the yaw differential.

## Implemented Fix

- Restored `CA_ROTOR*_CT=6.5`, matching PX4's multicopter allocator default and
  the working X-Plane eVTOL airframes. This keeps the yaw row safely above the
  allocator cutoff while preserving the existing `KM` signs.
- Reduced yaw demand for the first restored-authority test:
  `MC_YAW_P=0.4`, `MC_YAW_WEIGHT=0.3`, `MC_YAWRATE_P=0.06`,
  `MC_YAWRATE_I=0.01`, `MC_YAWRATE_MAX=35`.
- Updated hover takeoff defaults for the current 2.27 kg ACF:
  `MPC_THR_HOVER=0.25`, `MPC_TKO_RAMP_T=3`, `MIS_TAKEOFF_ALT=1.5`.
- Left estimator, sensor mapping, mass, propeller, battery, and motor geometry
  unchanged in this package.

## Physical Model Notes

The user's intended article is closer to a small Point Blank / ROC-X class
vehicle than to the current 5 lb Quantix-class ACF: approximately `6-6.5 kg`,
`0.7 m` span, 6S 11000 mAh battery, 10 inch tri-blade props, and MadMotr-class
3115 motors. Literature and product references support canted motors as a
reasonable tailsitter yaw/roll authority aid, often in the `5-10 deg` range.

That redesign should be its own slice after stable hover is recovered. It must
update the ACF mass/electrical/prop model, verify static thrust in
TruthCapture, then update PX4 control allocation axes to match the actual motor
cant. Applying it now would hide whether the allocator bug was fixed.

## Next Test

Use the v3.4.26 package with the updated `5021_xplane_qtailsitter`, run
`make distclean` once in PX4, then perform a hover-only test:

1. Take off to `1.5 m`.
2. Hold for `30 s`.
3. Apply only very small reposition or manual attitude inputs.
4. Land and wait `10-15 s` after disarm before stopping PX4.

Abort if yaw diverges by more than about `20 deg`, roll/pitch exceeds about
`15 deg`, an attitude warning appears, or the vehicle starts uncontrolled
climb/descent. Do not command forward transition yet.

## References

- PX4 control allocation source:
  `/home/alireza/PX4-Autopilot-Me/src/modules/control_allocator/ControlAllocator.cpp`
- PX4 control allocator defaults:
  `/home/alireza/PX4-Autopilot-Me/src/modules/control_allocator/module.yaml`
- X-Plane Plane Maker manual:
  `https://developer.x-plane.com/docs/aircraft/plane-maker-manual/`
- AeroVironment Quantix Recon article:
  `https://www.avinc.com/images/uploads/news/IUS_Quantix_Recon_Article.pdf`
- IAI ROC-X brochure:
  `https://www.iai.co.il/wp-content/uploads/2026/02/MSL-ROC-X-Brochure.pdf`
- Canted-motor tailsitter system-identification paper:
  `https://www.cambridge.org/core/services/aop-cambridge-core/content/view/334BCC740E5B20EA1B396050AD6606E2/S0001924025000351a.pdf/system-identification-of-a-hovering-quadrotor-biplane-tailsitter-with-canted-motors.pdf`
