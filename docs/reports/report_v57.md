# Report v57 - qtail21 MC Acceptance and First-Transition Gate

Date: 2026-05-26
Package target: `v3.4.45`

## Executive Summary

`qtail21.zip` is the first clean QuadTailsitter multicopter-mode acceptance run.
It used px4xplane `v3.4.44`, `Config Name: QuadTailsitter`, the packaged
`Aircraft/QuadTailsitter/QuadTailsitter.acf`, and the intended v3.4.44 PX4
airframe defaults. The stationary-ground bridge contract stayed clean: no baro
stale, vertical-velocity, high-accelerometer-bias, compass, or pause/resume
regression was found in the evidence reviewed.

The right next step is not another MC retune. v3.4.45 keeps the qtail21-proven
MC params and aircraft model unchanged, updates the report/test workflow, and
gates the next run toward one straight first transition before any FW orbit or
RTL tuning.

## Evidence Reviewed

From `/home/alireza/qtail21.zip`:

- TruthCapture recorded `22,696` rows over about `280.3 s` sim time, with `0`
  dropped rows, `0` sim-time resets, mean callback rate about `80 Hz`, p50
  frame rate about `81 Hz`, and `522` paused/sim-speed rows.
- X-Plane log loaded px4xplane `v3.4.44`, `Config Name: QuadTailsitter`,
  body-axis pitot `pitotAxisBody=-Z`, and the packaged QuadTailsitter aircraft.
- PX4 parameter sync was clean. The ULog matched the intended v3.4.44 values,
  including `MPC_XY_CRUISE=3.5`, `MPC_XY_VEL_MAX=4.0`,
  `MPC_ACC_HOR=0.7`, `MPC_ACC_HOR_MAX=1.0`, `MPC_JERK_AUTO=0.7`,
  `MPC_JERK_MAX=1.2`, `MPC_TILTMAX_AIR=30`, and `MC_PITCHRATE_D=0.0010`.
- ULog duration was about `217.2 s`. It did not contain a forward-transition
  attempt, so qtail21 is MC evidence only.
- MC horizontal speed stayed controlled: PX4 trajectory setpoint max was
  `3.5 m/s`, ULog horizontal speed max was about `4.1 m/s`, and TruthCapture
  true airspeed max was about `4.85 m/s`.
- Attitude remained inside the expected MC envelope: roll approximately
  `-4.9` to `8.9 deg`, pitch approximately `-15.8` to `7.5 deg`, and motor
  commands peaked around `0.345`.
- Estimator innovation ratios stayed low. ULog `timeout_flags` remained `0`.

## Landing Gear Note

Pressing `G` does not retract the big QuadTailsitter legs in the current
package because v3.4.43 intentionally made the four gear supports
non-retractable. That was a contact-stability change after earlier ground
jitter propagated into estimator warnings.

Do not make the physics gear retractable in v3.4.45. A cleaner future visual
solution is to keep stable fixed physics contact pads and add separate
retractable cosmetic legs or an object animation. That keeps the ground-contact
contract stable while allowing the in-flight visuals to hide later.

## v3.4.45 Decision

No QuadTailsitter PX4 params or ACF geometry are changed in this package.

Rationale:

- qtail21 validated the v3.4.44 MC speed, acceleration, jerk, tilt, hover, and
  contact baseline.
- The tiny hover/moving wobble reported by the user is acceptable for the first
  transition gate and should not be chased before FW evidence exists.
- PX4 VTOL guidance recommends starting transition work only after MC tuning is
  stable. The current evidence satisfies that gate.
- Existing FW/transition values are conservative for the 5 kg aircraft:
  `VT_ARSP_TRANS=24 m/s`, `FW_AIRSPD_TRIM=28 m/s`, large `900 m` loiter radii,
  and soft NPFG settings. They should be evaluated in one clean straight
  transition before any orbit/RTL tightening.

## Next Test

Use v3.4.45 for one first-transition data run:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean`.
2. Install the v3.4.45 px4xplane package and packaged QuadTailsitter aircraft.
3. Confirm X-Plane log shows `v3.4.45`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
4. Start TruthCapture before connecting PX4.
5. Run a stationary/pause sanity check, then MC takeoff, hover, and one Go-To.
6. If MC remains clean, climb high and command one straight forward transition.
7. Hold a short FW segment only; do not command FW orbit or RTL yet.
8. Manually back-transition and land. Wait `10-15 s` after disarm before
   stopping PX4.

Acceptance for the next run:

- no stationary/pause sensor warnings
- no MC overspeed repeat; actual horizontal speed below about `6 m/s` in MC
- transition reaches body-axis calibrated airspeed near `VT_ARSP_TRANS=24 m/s`
- no quad-chute, sustained motor saturation, uncontrolled sink, or divergent
  roll/pitch after FW stabilization
- FW throttle, pitch, airspeed, and altitude are stable enough to justify the
  next orbit/RTL tuning slice

## References

- PX4 multicopter jerk-limited trajectory:
  https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory
- PX4 multicopter PID tuning guide:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 VTOL/tailsitter overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter
- PX4 VTOL configuration and transition tuning:
  https://docs.px4.io/main/en/config_vtol/
- PX4 fixed-wing position/NPFG tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
