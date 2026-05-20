# Report v35 - QuadTailsitter qtail1 Crash Recovery

Scope: review `/home/alireza/qtail1.zip`, identify the hover crash root cause,
clean the QuadTailsitter naming, and prepare a hover-only recovery package.

## qtail1 Evidence

- PX4 ULog: `10_20_00.ulg`, `SYS_AUTOSTART=5021`, duration about `44.8 s`.
- XPlaneTruthCapture: `5,357` frames, mean callback about `82.3 Hz`, no dropped
  rows, no sim-time resets.
- X-Plane aircraft loaded the legacy-named QuadTailsitter `.acf` from the previous package.
- px4xplane config loaded: `QuadTailsitter`; four motor channels parsed.
- PX4 events:
  - armed at `t=28.15 s`
  - takeoff detected at `t=29.37 s`
  - roll attitude failure at `t=33.22 s`

The crash happened before transition. Fixed-wing, airspeed, TECS, NPFG, and
transition parameters were not exercised and should not be tuned from this log.

## Root Cause

The primary fault is a hover/control-allocation mismatch plus over-hot initial
multicopter tuning.

The previous airframe copied the X-Plane engine positions into PX4 as if they
were already PX4 body-frame meters. That was wrong for this aircraft:

- X-Plane aircraft coordinates: X is right, Y is up, Z is tail.
- PX4 control allocation expects body X forward and body Y right.
- Plane Maker ACF values are feet-scale aircraft values; the ULog/truth data
  confirms the aircraft mass is `2.27 kg` from the `5 lb` model.

The correct conversion for this model is:

- `PX4_X = -XPLANE_Z * 0.3048`
- `PX4_Y =  XPLANE_X * 0.3048`

The previous `CA_ROTOR*_PX/PY` values also made the model look much larger than
the aircraft in the allocator. In the log, once airborne, PX4 alternated between
diagonal motor pairs at saturation:

- left/right diagonal commands repeatedly hit `0.0` and `1.0`
- X-Plane roll reached roughly `+/-120 deg`
- roll rate exceeded `400-500 deg/s`
- pitch remained comparatively small

This is a fast roll-loop instability, not an estimator failure and not an FPS
issue.

## Changes Made

QuadTailsitter airframe:

- Corrected control-allocation rotor geometry to the converted X-Plane axes:
  - motor 0: `PX=0.22`, `PY=0.43`
  - motor 1: `PX=-0.22`, `PY=-0.43`
  - motor 2: `PX=0.22`, `PY=-0.43`
  - motor 3: `PX=-0.22`, `PY=0.43`
- Reduced multicopter attitude/rate aggressiveness for first safe hover:
  - `MC_ROLL_P=2.0`
  - `MC_PITCH_P=2.0`
  - `MC_ROLLRATE_P=0.18`
  - `MC_PITCHRATE_P=0.18`
  - `MC_ROLLRATE_K=0.45`
  - `MC_PITCHRATE_K=0.50`
  - `MC_ROLLRATE_MAX=120`
  - `MC_PITCHRATE_MAX=120`
- Softened takeoff and initial altitude control:
  - `MPC_THR_HOVER=0.22`
  - `MPC_USE_HTE=0`
  - `MPC_TKO_SPEED=1.0`
  - `MPC_TKO_RAMP_T=4.0`
  - `MPC_Z_V_AUTO_UP=1.2`
  - `MPC_TILTMAX_AIR=25`

Aircraft package:

- The release package now uses `QuadTailsitter/QuadTailsitter.acf`.
- The underlying ACF mass, motor, propeller, wing, and visual geometry are not
  changed in this slice. The evidence supports a PX4 allocation/tuning fault
  first, and changing aircraft physics at the same time would make the next log
  harder to interpret.

Naming:

- User-visible naming is now `QuadTailsitter`.
- The PX4 target remains `xplane_qtailsitter` and the airframe file remains
  `5021_xplane_qtailsitter` for compatibility with the existing PX4 build
  target.

## References Used

- PX4 Tailsitter VTOL docs:
  https://docs.px4.io/v1.14/en/frames_vtol/tailsitter.html
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter
- X-Plane aircraft coordinate documentation:
  https://developer.x-plane.com/article/screencoordinates/
- X-Plane force/moment coordinate documentation:
  https://developer.x-plane.com/article/movingtheplane/
- AeroVironment Quantix Recon datasheet for size context only:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf

## Next Test

Use v3.4.23 for a hover-only recovery test:

1. Set `config_name = QuadTailsitter`.
2. Run `make px4_sitl_default xplane_qtailsitter`.
3. Confirm `SYS_AUTOSTART=5021`.
4. Take off only to `2-3 m`.
5. Hold hover for at least `20 s`.
6. If stable, apply very small roll and pitch inputs.
7. Land and send ULog, terminal log, X-Plane Log.txt, and TruthCapture.

Do not test transition yet. The acceptance target for this slice is stable
hover without diagonal motor saturation, roll attitude failure, or uncontrolled
vertical motion.
