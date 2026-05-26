# Report v61 - qtail25 Acceptance Polish and v3.4.49 Fixes

Date: 2026-05-26

Package target: `v3.4.49`

## Summary

`qtail25.zip` is the first nearly complete QuadTailsitter mission validation:
takeoff, front transition, fixed-wing mission flight, back-transition, MC
descent, landing, disarm, and log close all completed. The bridge and aircraft
physics were not the primary failure source. The remaining defects were
fixed-wing yaw/roll wobble, a large climb after back-transition, transient
attitude-failure warnings, and early MC yaw oscillation during recovery/descent.

This slice keeps the X-Plane ACF unchanged and makes a scoped PX4 airframe
tuning pass.

## Evidence

- X-Plane loaded `px4xplane v3.4.48`, `Config Name: QuadTailsitter`, and
  body-axis pitot `-Z`.
- TruthCapture recorded `39,489` rows, `0` dropped rows, `0` sim-time resets,
  and about `82 Hz` mean callback rate.
- The PX4 ULog loaded the intended qtailsitter defaults; the only expected
  `check_px4_airframe_params.py` mismatch was `SYS_AUTOSTART`, which is set by
  the PX4 launch target rather than inside the airframe file.
- Fixed-wing cruise was credible: median TAS/CAS was about `29.5/29.3 m/s`,
  altitude stayed near `100 m`, TECS throttle p50 was about `0.11`, and throttle
  was not saturated.
- Fixed-wing lateral control still worked too hard: roll setpoint and lateral
  acceleration frequently hit the configured limit, with truth sideslip roughly
  `+/-8 deg`.
- Back-transition started around `98 m AGL` and `30.5 m/s`, ended around
  `91 m AGL` but still near `43 m/s`, then climbed to about `198 m AGL` during
  MC recovery. Truth thrust during the climb was not excessive; the climb was
  mainly stored forward-speed/aerodynamic energy plus the pitch-up recovery.
- Attitude warnings occurred immediately after transition back to MC, while the
  vehicle was still near a legal tailsitter recovery attitude.

## Design Check

The ACF remains plausible for the requested class:

- loaded truth mass: about `5.44 kg`
- empty mass: about `4.99 kg`
- wing area estimate: about `0.509 m^2`
- loaded wing loading: about `10.7 kg/m^2`
- stall estimate: about `12 m/s`
- fixed-wing trim target after this slice: `28 m/s`

That leaves more than `2x` stall margin in the first fixed-wing demo envelope.
The low fixed-wing throttle is a sign of a low-drag/high-power SITL model, but
qtail25 did not justify another mass, wing, motor, or prop redesign. Changing
the ACF now would mix physics and controller changes and make the successful
qtail25 evidence harder to preserve.

## Research Basis

- PX4 fixed-wing position tuning uses TECS for altitude/airspeed and NPFG for
  horizontal path tracking. That supports changing NPFG period, roll time
  constant, roll limit, and loiter radii for the observed path wobble:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing
- PX4 VTOL back-transition tuning describes `VT_B_TRANS_DUR` as the duration
  limit and `VT_B_DEC_MSS` as the mission deceleration estimate. That matches
  the qtail25 finding that back-transition geometry and entry energy, not bridge
  sensors, dominated the climb:
  https://docs.px4.io/main/en/config_vtol/vtol_back_transition_tuning
- PX4 parameter reference defines `FD_FAIL_R` and `FD_FAIL_P` as the roll/pitch
  attitude-failure limits. For this tailsitter SITL airframe, these are raised
  but not disabled:
  https://docs.px4.io/main/en/advanced_config/parameter_reference
- PX4 source review of `tailsitter.cpp` confirmed that tailsitter
  back-transition switches to MC when the pitch threshold is reached or
  `VT_B_TRANS_DUR` times out, and that the attitude setpoint rotates by
  `90 deg / VT_B_TRANS_DUR`.

## v3.4.49 Changes

- Reduced fixed-wing entry energy: `FW_AIRSPD_TRIM 29 -> 28`,
  `FW_AIRSPD_MIN 20 -> 19`, `FW_AIRSPD_MAX 40 -> 38`, `VT_ARSP_TRANS 23 -> 22`,
  and `VT_ARSP_BLEND 18 -> 17`.
- Smoothed fixed-wing yaw/roll coupling: `FW_R_LIM 17 -> 16`,
  `FW_R_TC 1.3 -> 1.5`, `FW_RR_P 0.12 -> 0.10`, `FW_RR_FF 0.06 -> 0.05`,
  `FW_YR_P 0.02 -> 0.01`, `VT_FW_DIFTHR_S_R 0.60 -> 0.55`, and
  `VT_FW_DIFTHR_S_Y 0.45 -> 0.32`.
- Enlarged guidance geometry: `NPFG_PERIOD 48 -> 55`,
  `NPFG_ROLL_TC 2.0 -> 2.4`, `NAV_ACC_RAD 90 -> 120`,
  `NAV_FW_ALT_RAD 30 -> 40`, and `NAV_LOITER_RAD/RTL_LOITER_RAD 1100 -> 1300`.
- Reduced recovery yaw aggression: `MC_YAW_P 0.80 -> 0.65`,
  `MC_YAWRATE_K 1.20 -> 1.10`, `MC_YAWRATE_MAX 65 -> 55`,
  `MPC_YAWRAUTO_MAX 35 -> 25`, and `MPC_YAWRAUTO_ACC 12 -> 8`.
- Gave back-transition more time without returning to the old long-dive setup:
  `VT_B_TRANS_DUR 4 -> 5` and `VT_B_DEC_MSS 1.1 -> 1.0`.
- Raised tailsitter failure-detector tolerance without disabling it:
  `FD_FAIL_R=85`, `FD_FAIL_P=85`, `FD_FAIL_R_TTRI=1.0`,
  `FD_FAIL_P_TTRI=1.0`.
- Sanitized the PX4 airframe comments so the PR branch contains a clean
  airframe definition, while the detailed qtail history remains in reports.

## Next Test

Use v3.4.49 with a clean PX4 parameter store. The next run can be a high-margin
mission test, but keep VTOL Land/back-transition altitude high. Expected
improvements are smaller FW yaw/roll wobble, lower recovery climb, no transient
attitude-failure warnings, and continued clean estimator/bridge behavior.

Send the ULog, X-Plane `Log.txt`, terminal output, and TruthCapture folder. The
first check after the run is whether `FW_AIRSPD_TRIM=28`, `VT_B_TRANS_DUR=5`,
`NAV_LOITER_RAD=1300`, and `FD_FAIL_P=85` loaded correctly.
