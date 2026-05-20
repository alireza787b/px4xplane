# Report v36 - QuadTailsitter qtail2 Hover Dynamics Recovery

Scope: review `/home/alireza/qtail2.zip`, identify why v3.4.23 still diverged
in hover, and prepare the next hover-only recovery package.

## qtail2 Evidence

- PX4 ULog: `15_08_36.ulg`, `SYS_AUTOSTART=5021`, duration about `257.5 s`.
- XPlaneTruthCapture: `9,138` frames, mean callback about `75.2 Hz`, median
  frame rate about `75.9 Hz`, zero dropped rows, and no sim-time resets.
- X-Plane log loaded px4xplane `v3.4.23`, `Config Name: QuadTailsitter`, and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- The qtail1 geometry fix was active in the ULog:
  `CA_ROTOR0_PX=0.22`, `CA_ROTOR0_PY=0.43`,
  `CA_ROTOR1_PX=-0.22`, `CA_ROTOR1_PY=-0.43`,
  `CA_ROTOR2_PX=0.22`, `CA_ROTOR2_PY=-0.43`,
  `CA_ROTOR3_PX=-0.22`, and `CA_ROTOR3_PY=0.43`.
- PX4 reported takeoff at `25.1 s`, then roll attitude failures starting at
  `36.7 s`. Later compass warnings follow the large attitude excursions and are
  treated as consequences, not the first cause.

## Finding

The remaining failure is not an FPS problem and not a broad px4xplane
sensor-contract regression. The vehicle already had the corrected motor
geometry, and the early hover outputs were not saturated:

- from `25-35 s`, motor outputs stayed roughly `0.13-0.39`
- estimated roll already moved from about `-35 deg` to `+19 deg`
- roll-rate p99 reached about `136 deg/s`
- roll/pitch setpoints and motor differentials had the expected correction
  direction, but the aircraft overshot and reversed with a growing slow
  oscillation

That points to an under-damped hover plant/controller combination. Two
contributors were found:

1. The X-Plane aircraft still had Plane Maker Artificial Stability engine
   application enabled for all four motors:
   `acf/_FBW_apply_P/Q/R_to_engine/0..3 = 1`, with low-speed rate damping
   values present.
2. The v3.4.23 PX4 hover tune was still too aggressive for this very light
   `5-6 lb` X-Plane model after the geometry correction.

The Plane Maker manual describes Artificial Stability as adding inputs on top
of pilot controls. For an externally controlled SITL vehicle, px4xplane should
have exclusive authority over the motor datarefs. Leaving X-Plane's internal
engine stabilization available makes the model harder to reason about and can
fight or mask PX4 control-loop behavior.

## Land-Detector Warning

The terminal warning:

`LNDMC_Z_VEL_MAX > MPC_LAND_CRWL or MPC_LAND_SPEED, updating 0.300 -> 0.250`

was caused by our own parameter boundary. PX4 land-detector guidance requires
`MPC_LAND_CRWL` to be larger than `LNDMC_Z_VEL_MAX`. v3.4.24 sets
`LNDMC_Z_VEL_MAX=0.20` to remove the startup correction.

## Changes Made

Aircraft:

- Added `aircraft/QuadTailsitter/` to the px4xplane repo so future packages are
  built from a source-controlled aircraft, not a hidden local folder.
- Disabled X-Plane internal artificial-stability engine mixing for the four
  external-autopilot motors:
  - `FBW_apply_P_to_engine/0..3 = 0`
  - `FBW_apply_Q_to_engine/0..3 = 0`
  - `FBW_apply_R_to_engine/0..3 = 0`
- Set the low-speed artificial-stability rate gains and rate limits to zero.

PX4 airframe:

- Disabled `MC_AIRMODE` for this hover-recovery slice, matching PX4's tuning
  guidance while first-takeoff oscillations are being reduced.
- Reduced MC attitude and rate gains:
  - `MC_ROLL_P=0.8`, `MC_PITCH_P=0.8`
  - `MC_ROLLRATE_P=0.08`, `MC_PITCHRATE_P=0.08`
  - `MC_ROLLRATE_K=0.30`, `MC_PITCHRATE_K=0.30`
  - `MC_ROLLRATE_MAX=70`, `MC_PITCHRATE_MAX=70`
- Reduced yaw demand during hover recovery:
  - `MC_YAW_P=0.4`, `MC_YAW_WEIGHT=0.2`
  - `MC_YAWRATE_P=0.06`, `MC_YAWRATE_MAX=45`
- Reduced takeoff, horizontal correction, acceleration, jerk, and tilt limits so
  the next log can validate the inner hover loop before transition testing:
  - `MPC_TKO_SPEED=0.6`, `MPC_TKO_RAMP_T=5`
  - `MPC_TILTMAX_AIR=15`, `MPC_MAN_TILT_MAX=15`
  - `MPC_XY_CRUISE=3`, `MPC_XY_VEL_MAX=4`
  - `MPC_ACC_HOR=1.5`, `MPC_JERK_AUTO=2`
- Set `LNDMC_Z_VEL_MAX=0.20`.

## References Used

- PX4 Tailsitter VTOL docs:
  https://docs.px4.io/v1.13/en/frames_vtol/tailsitter
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter
- PX4 land detector configuration:
  https://docs.px4.io/main/en/advanced_config/land_detector
- X-Plane Plane Maker manual, Artificial Stability section:
  https://developer.x-plane.com/manuals/planemaker/

## Next Test

Use v3.4.24 for one hover-only recovery test:

1. Install `X-Plane_Aircraft/QuadTailsitter`.
2. Install `PX4_airframes_copy_to_ROMFS/5021_xplane_qtailsitter`.
3. Run `make distclean` once, then `make px4_sitl_default xplane_qtailsitter`.
4. Start XPlaneTruthCapture.
5. Take off only to about `2 m`.
6. Hold for `20 s`; do not command transition.
7. Abort if roll or pitch exceeds about `15 deg` or attitude warnings appear.

Acceptance target: no growing roll/pitch oscillation, no attitude failure, no
diagonal motor saturation, no startup land-detector parameter correction, and a
clean landing/disarm.
