# Report v53 - Ground-Stationary Kinematics Guard

Date: 2026-05-25
Package target: `v3.4.41`

## Executive Summary

v3.4.40 reduced the immediate vertical-velocity instability, but did not remove
it for QuadTailsitter. That means the acceleration guard was only part of the
problem. X-Plane can also report vertical velocity while the aircraft is still
stationary on its landing gear, and that velocity was still being sent to PX4
through HIL_GPS and HIL_STATE_QUATERNION.

Alia showing smaller vertical-speed spikes without the PX4 warning is important
evidence: this is not only a QuadTailsitter tuning problem. The bridge needs a
general stationary-ground kinematics contract, while the QuadTailsitter aircraft
contact model should be reviewed separately because it appears to amplify the
jitter.

## Diagnosis

The v3.4.40 guard was too narrow:

- It guarded HIL_SENSOR acceleration only.
- It required X-Plane vertical speed to stay below `0.5 m/s`.
- If a landing-gear/contact bounce produced the actual vertical-speed spike, the
  guard disabled itself for that frame.

That explains why the problem became smaller but remained visible.

## Implemented Changes

- Added `ground_stationary_kinematics_guard_enabled = true`.
- Reused a general stationary-on-ground detector based on:
  - X-Plane on-ground flag
  - low AGL
  - low horizontal speed
- Removed vertical-speed from the guard gate, because vertical-speed spikes are
  the artifact being suppressed.
- While the guard is active, px4xplane now sends zero local NED velocity to:
  - HIL_GPS `vn`, `ve`, and `vd`
  - GPS course-over-ground update logic
  - HIL_STATE_QUATERNION `vx`, `vy`, and `vz`
  - body-axis pitot projection input velocity
- HIL_SENSOR acceleration still uses stable FRD gravity while the guard is
  active.

This is not hardcoded to QuadTailsitter or Alia. It applies only to the
stationary-on-ground simulator contact state and disengages once the aircraft is
moving horizontally or airborne.

## QuadTailsitter Landing-Gear Note

The current QuadTailsitter ACF has an unusual contact model: long angled gear
with large strut compression values and very small tire/contact dimensions. That
can make X-Plane contact dynamics noisier than Alia. I did not change the ACF in
this hotfix because the same symptom exists at smaller amplitude on Alia, so the
bridge sensor contract must be fixed first.

If v3.4.41 removes the PX4 warning but the truth data still shows excessive
on-ground bounce, the next aircraft-side slice should tune the QuadTailsitter
landing gear/contact model in Plane Maker or the ACF.

## Test Instruction

1. Install v3.4.41.
2. Start with a stationary connect test on QuadTailsitter and Alia.
3. Confirm X-Plane `Log.txt` contains:

   ```text
   px4xplane: Version: v3.4.41
   px4xplane: Ground-stationary accelerometer guard enabled
   px4xplane: Ground-stationary kinematics guard enabled
   px4xplane: [GROUND_ACCEL_GUARD]
   px4xplane: [GROUND_KINEMATICS_GUARD]
   ```

4. If PX4 is clean while stationary, resume QuadTailsitter MC testing.
5. If warnings remain, collect ULog plus TruthCapture from the stationary case
   before changing aircraft tuning.
