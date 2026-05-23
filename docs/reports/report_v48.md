# Report v48 - QuadTailsitter qtail13 FW Energy and Orbit Recovery

Date: 2026-05-23  
Package target: `v3.4.36`

## Inputs

- `/home/alireza/qtail13.zip`
- PX4 ULog: `19_18_23.ulg`
- X-Plane log: `Log.txt`
- TruthCapture run: `20260523-191720Z`

The zip is valid. TruthCapture recorded `42,479` frames over about `518 s` of
sim time with zero dropped rows, zero sim-time resets, about `82.1 Hz` mean
callback rate, and `50 ms` worst frame period.

## What Was Not Wrong

qtail13 did not use stale parameters and did not use open-loop transition.
The ULog active defaults match the v3.4.35 qtail file, including:

- `FW_USE_AIRSPD=1`
- `SYS_HAS_NUM_ASPD=1`
- `VT_ARSP_TRANS=18`
- `FW_AIRSPD_TRIM=22`
- `NAV_LOITER_RAD=500`
- `RTL_LOITER_RAD=500`
- `airspeed_validated.airspeed_source=1`

The bridge also loaded the intended aircraft/config:

- px4xplane `v3.4.35`
- `Config Name: QuadTailsitter`
- `Airspeed source=body_axis pitotAxisBody=-Z`
- `Aircraft/QuadTailsitter/QuadTailsitter.acf`

## qtail13 Root Cause

The first quad-chute was a fixed-wing minimum-altitude breach, not an airspeed
or attitude mapping failure. Transition was commanded with little margin above
`VT_FW_MIN_ALT=40 m`; local altitude crossed just below `40 m`, and PX4 set the
VTOL fixed-wing failure flag. TECS was still trying to climb toward the takeoff
altitude setpoint.

The later FW orbit/RTL failure was an energy and path-capture problem:

- FW loiter calibrated airspeed: mean about `35.0 m/s`, p95 about `41.9 m/s`
- FW RTL calibrated airspeed: mean about `37.6 m/s`, p95 about `42.0 m/s`
- TECS true airspeed target stayed near `22.1 m/s`
- Lateral acceleration repeatedly hit `6.87 m/s^2`, the configured
  `35 deg` roll-limit equivalent
- Actual transformed FW bank followed the saturated request; median bank was
  about `34-35 deg` in the useful FW segments
- The actual RTL fit radius was roughly `1.3 km`, so the vehicle was not close
  to the commanded `500 m` path

The important new aircraft-model finding is that the ACF propellers were still
designed for `179 kt`. That is inconsistent with the Quantix-class target. The
official Quantix Recon datasheet lists `5 lb (2.3 kg)`, `3.2 ft (97.5 cm)`
wingspan, four direct electric motors, and 20 km round-trip/40 km one-way
mission coverage. An AeroVironment-hosted article lists speed as `45 mph`,
which is about `20 m/s`. PX4's own quadtailsitter SITL example uses
`FW_AIRSPD_TRIM=18` and `FW_AIRSPD_MAX=22`.

So the current mass and span are plausible, but the prop design speed is not.
At the achieved speed, even low throttle and differential-thrust corrections
produce enough net forward thrust to keep the vehicle fast. Tightening guidance
then requests more bank/torque, which adds more propulsive energy and makes
the path capture worse.

## Changes Made

QuadTailsitter X-Plane aircraft:

- `P _prop/*/_des_kts_acf: 179 -> 40`
- `P _prop/*/_des_rpm_prp: 28916 -> 16000`
- `P _engn/*/_RSC_max_pwr_ENGN: 29896 -> 22000`
- `P _engn/*/_RSC_redline_ENGN: 29896 -> 22000`

The aircraft remains `5 lb / 2.27 kg`; that matches the Quantix-class public
spec and the previous hover logs. Jumping to `5 kg` would require a larger
motor/prop/wing redesign.

PX4 QuadTailsitter defaults:

- Speed envelope: `FW_AIRSPD_STALL=11`, `FW_AIRSPD_MIN=14`,
  `FW_AIRSPD_TRIM=20`, `FW_AIRSPD_MAX=28`
- Throttle/energy: `FW_THR_TRIM=0.05`, `FW_THR_MAX=0.25`,
  `FW_THR_SLEW_MAX=0.20`, `FW_T_SPDWEIGHT=2.0`, `FW_T_RLL2THR=2.0`,
  `FW_T_THR_DAMPING=0.25`, `FW_T_PTCH_DAMP=0.25`, `FW_PSP_OFF=4`
- Bank/guidance: `FW_R_LIM=28`, `NPFG_PERIOD=34`,
  `NPFG_DAMPING=1.0`, `NPFG_ROLL_TC=1.5`,
  `NAV_LOITER_RAD=900`, `RTL_LOITER_RAD=900`
- VTOL safety/RTL: `VT_FW_MIN_ALT=30`, `RTL_RETURN_ALT=100`,
  `RTL_DESCEND_ALT=75`

## Rationale

PX4 TECS tuning guidance says trim airspeed, trim throttle, pitch offset, and
airspeed/throttle limits must come from the actual vehicle's level-flight
behavior, not desired behavior. qtail13 proved the aircraft model was not
consistent with its intended vehicle class, so changing only TECS would be
patchy.

PX4 NPFG parameters represent the fixed-wing path-following dynamics. qtail13
showed repeated lateral acceleration saturation, so the next test uses a larger
loiter radius, a longer NPFG period, stronger damping, and a lower roll limit.
The goal is to break the overspeed -> roll saturation -> differential thrust
-> more speed loop before trying to tighten the radius again.

## Next Test

Use v3.4.36 for one controlled run:

1. Install the packaged QuadTailsitter aircraft and px4xplane plugin.
2. Pull the PX4 `px4xplane-sitl` branch and force a clean parameter load
   (`make distclean` is safest).
3. Confirm X-Plane log shows `v3.4.36`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
4. Confirm the ULog active params include `FW_AIRSPD_TRIM=20`,
   `FW_T_SPDWEIGHT=2.0`, `FW_R_LIM=28`, `NAV_LOITER_RAD=900`, and
   `RTL_LOITER_RAD=900`.
5. Fly MC takeoff, Hold, 3 m/s Go-To, 4-5 m/s Go-To, and a 40-50 m MC Orbit.
6. Climb to at least `80 m` AGL before forward transition.
7. After FW transition, hold straight for `10-15 s` before judging loiter.
8. Test FW loiter with at least `900 m` radius.
9. Try RTL only after FW loiter speed and bank look stable.

Acceptance for the next log:

- FW calibrated/TAS settles closer to `20-28 m/s`, not the high `30s`/low
  `40s`.
- FW path following no longer spends long periods saturated at roll limit.
- The first transition should not quad-chute unless commanded too low or the
  vehicle genuinely loses more than the configured altitude-loss threshold.
- RTL should capture a wide return loiter instead of diverging into kilometer
  scale path error.

## References

- PX4 fixed-wing altitude/position tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing.html
- PX4 parameter reference:
  https://docs.px4.io/main/en/advanced_config/parameter_reference.html
- PX4 tailsitter overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter.html
- AeroVironment Quantix Recon datasheet:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf
- AeroVironment-hosted Quantix Recon article:
  https://www.avinc.com/images/uploads/news/IUS_Quantix_Recon_Article.pdf
