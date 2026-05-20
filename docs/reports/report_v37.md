# Report v37 - QuadTailsitter qtail3 Yaw Authority Recovery

Scope: review `/home/alireza/qtail3.zip`, identify the remaining slow hover
divergence, check the long connection wait, and prepare the next
QuadTailsitter hover package.

## qtail3 Evidence

- PX4 ULog: `15_54_19.ulg`, `SYS_AUTOSTART=5021`, duration about `68.0 s`.
- XPlaneTruthCapture: `9,261` frames, mean callback about `84.0 Hz`, zero
  dropped rows, and no sim-time resets.
- X-Plane loaded px4xplane `v3.4.24`, `Config Name: QuadTailsitter`, and the
  source-controlled `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- Truth data confirmed the model mass stayed at `2.27 kg`, matching the
  Quantix-class `5 lb / 2.3 kg` reference size. Wind at the aircraft was about
  `0.6-1.1 m/s`, which is not enough to explain the failure by itself.
- PX4 detected takeoff at `19.36 s`, then reported pitch attitude failure at
  `53.63 s`, roll attitude failure at `55.71 s`, and blind-land failsafe at
  `63.59 s`.

## Finding

qtail3 fixed the violent first-lift oscillation. The remaining failure is a yaw
authority problem that then turns into position drift and attitude loss.

From about `30-53 s`, motors were not saturated (`0.21-0.31` normalized), but:

- yaw setpoint held near `137.6 deg`
- actual yaw drifted to about `65 deg`
- yaw error grew to roughly `70 deg`
- PX4 commanded up to about `28 deg/s` yaw-rate correction
- `control_allocator_status` reported yaw torque as unallocated
- actuator motor outputs had effectively zero current-KM yaw differential

This is the expected downside of leaving `MC_AIRMODE=0` after the roll/pitch
recovery pass. Disabling airmode helped reduce the qtail2 oscillation while the
inner loop was too hot, but qtail3 shows the reduced gains are now calm enough
to restore yaw allocation authority.

The connection wait was not a v3.5.20-style telemetry regression. X-Plane opened
the simulator TCP server, waited about `25 s`, then accepted PX4. After that,
PX4 logged startup success and became ready for takeoff in about `2.3 s`.

## Changes Made

QuadTailsitter airframe:

- Restored roll/pitch/yaw airmode:
  - `MC_AIRMODE=2`
  - `MAN_ARM_GESTURE=0`
- Kept the reduced roll/pitch tune from qtail3.
- Raised yaw response moderately:
  - `MC_YAW_P=0.6`
  - `MC_YAW_WEIGHT=0.4`
  - `MC_YAWRATE_P=0.10`
  - `MC_YAWRATE_I=0.03`
  - `MC_YAWRATE_MAX=60`
- Set `MIS_TAKEOFF_ALT=3.0` so the first hover test does not inherit the VTOL
  default `20 m` takeoff altitude.
- Set `LNDMC_Z_VEL_MAX=0.25`, matching the actual PX4 clamp observed in the
  ULog while remaining below `MPC_LAND_CRWL=0.30`.

Aircraft:

- No `.acf` physics change in this slice. The evidence points to yaw allocation
  and takeoff-altitude defaults, not mass, CG, motor placement, or airfoil
  geometry. Changing aircraft physics at the same time would hide whether the
  allocation fix worked.

## References Used

- PX4 multicopter PID tuning guide:
  https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter
- PX4 takeoff mode parameters:
  https://docs.px4.io/main/en/flight_modes_mc/takeoff
- PX4 control allocation docs:
  https://docs.px4.io/v1.16/en/concept/control_allocation
- PX4 parameter reference for rotor `KM` direction:
  https://docs.px4.io/v1.15/en/advanced_config/parameter_reference
- AeroVironment Quantix Recon datasheet for size context:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf

## Next Test

Use v3.4.25 for one hover-only test:

1. Install `X-Plane_Aircraft/QuadTailsitter`.
2. Install `PX4_airframes_copy_to_ROMFS/5021_xplane_qtailsitter`.
3. Run `make distclean` once after replacing the airframe, then run
   `make px4_sitl_default xplane_qtailsitter`.
4. Start XPlaneTruthCapture.
5. Take off to `3 m`, hold for at least `30 s`, then land.
6. Abort if heading error grows past about `20 deg`, roll/pitch exceeds about
   `15 deg`, or attitude/failsafe warnings appear.

Acceptance target: stable hover with yaw tracking, no growing horizontal drift,
no unallocated yaw torque buildup, no diagonal motor saturation, and clean
landing/disarm.
