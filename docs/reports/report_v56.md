# Report v56 - qtail20 MC Overspeed Recovery and 5 kg Design Audit

Date: 2026-05-25
Package target: `v3.4.44`

## Executive Summary

qtail20 confirms the v3.4.43 bridge-side stationary fixes worked. The run used
px4xplane `v3.4.43`, `Config Name: QuadTailsitter`, body-axis pitot `-Z`, and
the stationary-ground contract. The X-Plane log showed pause/resume handling and
did not show the earlier baro stale, vertical velocity, high accelerometer bias,
or compass failures.

The remaining problem is vehicle tuning. During the first MC Go-To, PX4 did not
command a `14 m/s` cruise speed: `trajectory_setpoint` stayed at or below
`5 m/s`, but actual horizontal speed reached `13.8 m/s` in ULog and `14.4 m/s`
in TruthCapture. The attitude response overshot badly; around peak speed the
vehicle was still near `-55 deg` pitch while the position controller had already
started braking.

Important caveat: qtail20 did not load the already-pushed v3.4.43 PX4 qtail
softening. The ULog still had `MPC_ACC_HOR=1.2`, `MPC_ACC_HOR_MAX=1.8`,
`MPC_JERK_AUTO=1.2`, `MPC_JERK_MAX=2.0`, and `LNDMC_Z_VEL_MAX=0.30`. The
current branch expected `1.0`, `1.5`, `1.0`, `1.8`, and `0.25`. The next test
must force a clean PX4 parameter load before judging the tune.

## Evidence Reviewed

From `/home/alireza/qtail20.zip`:

- TruthCapture recorded `16,211` frames, zero dropped rows, zero sim-time resets,
  mean callback rate about `82 Hz`, p50 frame rate about `83 Hz`, p95 frame
  period about `14.5 ms`, and `1003` paused rows.
- X-Plane log loaded `Aircraft/QuadTailsitter/QuadTailsitter.acf` and
  `px4xplane: Version: v3.4.43`.
- The bridge logged `[BRIDGE_PAUSE]` and `[BRIDGE_RESUME]` and then released the
  stationary sensor contract after takeoff.
- ULog showed no transition attempt in this run; it was useful for MC only.
- First MC Go-To:
  - PX4 `trajectory_setpoint` horizontal speed p95 was `5.0 m/s`.
  - PX4 `vehicle_local_position` horizontal speed max was `13.8 m/s`.
  - TruthCapture horizontal speed max was `14.4 m/s`.
  - Actual pitch reached about `-55 deg` near peak speed.
  - Motor outputs were not maxed; observed motor command peak was about `0.37`.
  - Control allocation did not show sustained torque/thrust saturation.

## 5 kg Design Audit

The current ACF is internally plausible for the chosen `5 kg` demo target:

- Truth loaded mass: about `5.44 kg`.
- ACF empty/max mass: about `4.99 kg` / `5.67 kg`.
- Wing estimate: about `0.509 m^2`; wing loading roughly `105 N/m^2` at
  qtail20 loaded mass.
- Stall estimate remains around `11-12 m/s` for `CLmax=1.2`, so a conservative
  fixed-wing transition target above `24 m/s` remains defensible.
- Propellers are `~9.8 in`, three blade, loaded design RPM `17,000`, with a
  redline compatible with the 6S/1000 KV class.

No broad ACF rewrite is justified from qtail20. The aircraft no longer behaves
like the old too-light plant, and the immediate failure is not thrust deficit or
landing contact. Keep the ACF unchanged in v3.4.44 and retest with a cleaner MC
trajectory envelope first. Reopen ACF geometry only if the next clean-parameter
run still shows attitude lag with conservative setpoints.

## Implemented Changes

QuadTailsitter PX4 defaults:

- `MPC_XY_CRUISE: 5.0 -> 3.5`
- `MPC_XY_VEL_MAX: 5.0 -> 4.0`
- `MPC_VEL_MANUAL: 5.0 -> 4.0`
- `MPC_ACC_HOR: 1.0 -> 0.7`
- `MPC_ACC_HOR_MAX: 1.5 -> 1.0`
- `MPC_JERK_AUTO: 1.0 -> 0.7`
- `MPC_JERK_MAX: 1.8 -> 1.2`
- `MPC_MAN_TILT_MAX: 35 -> 30`
- `MPC_TILTMAX_AIR: 40 -> 30`
- `MPC_THR_XY_MARG: 0.3 -> 0.4`
- `MC_PITCHRATE_D: 0.0008 -> 0.0010`

Rationale: PX4's jerk-limited auto trajectory can plan a capped velocity, but
the vehicle still has to track the attitude and acceleration. qtail20 showed
large pitch transient error rather than a speed-command bug. v3.4.44 therefore
reduces auto speed, acceleration, jerk, and tilt together, and adds only a tiny
pitch-rate damping increase. It does not retune the entire attitude controller.

## Next Test

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean`.
2. Install v3.4.44 plugin and packaged QuadTailsitter aircraft.
3. Confirm the ULog parameters match v3.4.44 before judging flight.
4. Stationary/pause sanity check for `60-90 s`.
5. MC only first:
   - takeoff and hold
   - one small Go-To
   - one longer Go-To
   - RTL or land
6. Acceptance before transition:
   - no sensor/estimator warnings
   - actual horizontal speed does not exceed about `6 m/s`
   - pitch does not run away beyond about `35 deg` in MC Go-To
   - no sustained motor saturation
   - landing contact and disarm remain clean
7. Only if MC passes, climb high and do one straight first transition. Do not
   test FW orbit/RTL yet.

## References

- PX4 jerk-limited multicopter trajectory:
  https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter
- PX4 VTOL/tailsitter overview:
  https://docs.px4.io/v1.14/en/frames_vtol/tailsitter.html
- PX4 VTOL configuration:
  https://docs.px4.io/v1.14/en/config_vtol/
- AeroVironment Quantix Recon public reference:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf
