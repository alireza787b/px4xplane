# Report v49 - QuadTailsitter qtail14 5 kg Retarget and Airspeed Prearm Cleanup

Date: 2026-05-24  
Package target: `v3.4.37`

## Inputs

- `/home/alireza/qtail14.zip`
- PX4 ULog: `02_32_36.ulg`
- X-Plane log: `Log.txt`
- TruthCapture run: `20260524-023840Z`

The archive is valid. TruthCapture recorded `12,300` frames over about `148.5 s`
of sim time with zero dropped rows and zero sim-time resets. Mean callback rate
was about `82.9 Hz`; p95 frame period was about `14.5 ms`.

## What Was Active

The run used the intended current software stack:

- px4xplane `v3.4.36`
- PX4 fork commit `ad499de`
- `SYS_AUTOSTART=5021`
- `Config Name: QuadTailsitter`
- `Airspeed source=body_axis pitotAxisBody=-Z`
- `airspeed_validated.airspeed_source=1`

So this was not a stale-params or airspeedless-transition case.

## Airspeed Selector Warning

The startup warning:

```text
Preflight Fail: Airspeed selector module down
```

comes from PX4 commander health checks when `SYS_HAS_NUM_ASPD > 0` and no fresh
`airspeed_validated` topic is available. PX4 starts commander before the full
vehicle app stack is warm. `airspeed_selector` is also intentionally delayed for
about two seconds after boot. With a simulator pitot, this makes
`SYS_HAS_NUM_ASPD=1` a startup race and a noisy user experience.

This does not mean the modeled pitot is unusable. qtail14 logged valid raw
airspeed, differential pressure, and `airspeed_validated` data after startup.
The clean X-Plane SITL policy is:

- keep `FW_USE_AIRSPD=1` where the aircraft needs pitot feedback,
- keep `ASPD_FALLBACK=1`,
- use at least the missing-data check where appropriate,
- set `SYS_HAS_NUM_ASPD=0` so the virtual pitot is not a prearm hardware
  requirement.

This matches the already-successful Alia pattern and was applied to
QuadTailsitter, Cessna 172, and TB2.

## qtail14 Flight Root Cause

qtail14 was chaotic mainly because the X-Plane aircraft still had the old mass:

- Truth `sim/flightmodel/weight/m_total`: `2.2679 kg`
- ACF `acf/_m_empty`: `5.0 lb`
- Intended target from the current design discussion: about `5 kg`

The climb evidence is direct:

- PX4 takeoff/MC phase altitude rose from ground to about `577 m`
- PX4 altitude setpoint during that segment was only about `37 m`
- Truth AGL reached about `586 m` before RTL
- Vertical speed was commonly `4-5 m/s` in MC and reached much higher during
  transition/FW
- Per-motor command around `0.12-0.13` was already close to hover thrust for
  the `2.27 kg` model

This explains why lowering throttle did not stop the climb: the old airframe
was too light for the requested 5 kg design target. A parameter-only patch would
continue fighting the wrong plant.

## Motor and Battery Retarget

The ACF has been retargeted to the requested 6S quad tailsitter class:

- Mass: `5.0 lb -> 11.0 lb` empty, `6.0 lb -> 12.5 lb` max
- Battery: `64 V / 120 Wh / 60 A -> 22.2 V / 244 Wh / 180 A`
- Motor redline/max RPM: `22,000 -> 25,000`
- Prop loaded design RPM: `16,000 -> 17,000`
- Prop design speed: `40 kt -> 50 kt`
- Motor power limit: `1.74 -> 2.40 hp` per motor

This does not simply "make KV lower". v3.4.36 had reduced the redline to slow
the old 2.27 kg aircraft. v3.4.37 corrects the target: the no-load/redline RPM
is now consistent with a 1000 KV motor on a 6S pack near full charge. The prop
design RPM remains lower than no-load RPM because a loaded 10 inch prop should
not be modeled as spinning at ideal no-load speed.

For hover math: a 5 kg vehicle needs about `49 N` total thrust, or about
`12.3 N` per motor. The next log should verify actual hover throttle and static
thrust; expected hover target is roughly the `0.25-0.35` command range, not the
old `0.12-0.13` range.

## Motor Cant

A first 5 degree mirrored motor-cant test was added in both places:

- X-Plane ACF: engine `part_psi` signs mirrored by motor pair
- PX4 control allocation: rotor axes include matching XY components

The PX4 axes are:

```sh
CA_ROTOR0_AX -0.078
CA_ROTOR0_AY  0.040
CA_ROTOR0_AZ -0.996

CA_ROTOR1_AX  0.078
CA_ROTOR1_AY -0.040
CA_ROTOR1_AZ -0.996

CA_ROTOR2_AX -0.078
CA_ROTOR2_AY -0.040
CA_ROTOR2_AZ -0.996

CA_ROTOR3_AX  0.078
CA_ROTOR3_AY  0.040
CA_ROTOR3_AZ -0.996
```

This is intentionally small. It should improve yaw authority in MC mode and
roll/yaw authority in the transformed fixed-wing mode without relying only on
prop torque. Because X-Plane ACF engine-angle sign conventions are less
transparent than PX4 rotor axes, the next test must include a hover yaw check
before transition.

## PX4 Parameter Changes

QuadTailsitter changes for the heavier model:

- `SYS_HAS_NUM_ASPD=0`, `ASPD_DO_CHECKS=1`
- `MPC_THR_HOVER=0.30`, `MPC_THR_MIN=0.08`
- `MPC_TKO_RAMP_T=2.5`
- `MPC_Z_VEL_MAX_UP=1.2`, `MPC_ACC_UP_MAX=1.6`
- `MPC_ACC_HOR=1.2`, `MPC_ACC_HOR_MAX=1.8`
- `FW_AIRSPD_STALL=16`, `FW_AIRSPD_MIN=21`, `FW_AIRSPD_TRIM=28`,
  `FW_AIRSPD_MAX=40`
- `FW_THR_TRIM=0.16`, `FW_THR_MAX=0.40`, `FW_T_SPDWEIGHT=1.5`
- `VT_ARSP_BLEND=18`, `VT_ARSP_TRANS=24`, `VT_F_TRANS_THR=0.55`

The next run should be treated as a plant-retarget validation, not a final
mission demo.

## Next Test

1. Pull the PX4 branch and force a clean parameter load with `make distclean`.
2. Install the v3.4.37 px4xplane plugin and the packaged `QuadTailsitter`
   aircraft.
3. Start XPlaneTruthCapture before connecting PX4.
4. Confirm in X-Plane `Log.txt`:
   - `px4xplane: Version: v3.4.37`
   - `Config Name: QuadTailsitter`
   - `Airspeed source=body_axis pitotAxisBody=-Z`
5. Confirm in TruthCapture or X-Plane data output:
   - `sim/flightmodel/weight/m_total` is about `5 kg`
6. First flight should be MC-only:
   - takeoff,
   - hold,
   - short 2-3 m/s Go-To,
   - yaw left/right checks,
   - land.
7. Only if hover/yaw/altitude are clean, do a second run with:
   - 4-5 m/s Go-To,
   - 40-50 m MC orbit,
   - climb to at least `100 m` AGL,
   - one forward transition into open space.

Acceptance for the next log:

- No recurring airspeed-selector prearm warning after the virtual-pitot policy
  change.
- Ground truth mass is about `5 kg`.
- MC hover no longer climbs through altitude setpoint.
- Hover throttle is in a plausible range for the new plant.
- Yaw response improves or at least does not reverse with the new cant signs.
- If transition is attempted, body-axis calibrated airspeed should pass
  `24 m/s` before fixed-wing completion.

## References

- PX4 airspeed calibration and sensor setup:
  https://docs.px4.io/main/en/config/airspeed
- PX4 airspeed sensors:
  https://docs.px4.io/v1.16/en/sensor/airspeed
- PX4 tailsitter overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter.html
- PX4 actuator configuration:
  https://docs.px4.io/main/en/config/actuators
- PX4 fixed-wing position tuning:
  https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing.html
- ArduPilot tailsitter tuning background:
  https://ardupilot.org/plane/docs/tailsitter-tuning-guide.html
