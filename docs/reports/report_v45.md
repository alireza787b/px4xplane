# Report v45 - QuadTailsitter qtail10 FW Energy Recovery

Date: 2026-05-22

## Scope

This report reviews `/home/alireza/qtail10.zip`, answers the pitot/tailsitter
sensor-frame question, and defines the next QuadTailsitter package after the
first successful but poorly controlled fixed-wing transition.

## Log Integrity

- Zip test: OK.
- PX4 ULog: `19_44_49.ulg`, about `520.6 s`.
- XPlaneTruthCapture: `45,719` frames, `0` dropped rows, mean callback rate
  about `80 Hz`, max frame period about `50 ms`.
- X-Plane loaded px4xplane `v3.4.32` and `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- The PX4 airframe defaults were the intended v3.4.32 values:
  `FW_USE_AIRSPD=0`, `VT_ARSP_TRANS=0`, `VT_F_TRANS_DUR=6`,
  `VT_F_TRANS_THR=0.65`, `VT_FW_MIN_ALT=40`, and `VT_QC_T_ALT_LOSS=35`.

## qtail10 Findings

The first transition failure was expected from the configured safety margin:
the attempt started around `26 m` AGL while `VT_FW_MIN_ALT=40`. PX4 quad-chuted
and entered the normal failsafe sequence. Do not treat that as an airspeed or
bridge failure.

The second transition reached fixed-wing state at about `141 s`, but the FW
segment was not acceptable:

- Truth and ULog show FW groundspeed/TAS mostly around `40-60 m/s`.
- PX4 was airspeedless, so TECS used its synthetic trim value around `18 m/s`.
- The commanded FW loiter radius was `120 m`, but at `50 m/s` and `45 deg`
  bank the physical minimum turn radius is about `255 m`.
- The FW path therefore became large and sloppy; fitted truth radii were in the
  hundreds of meters.
- Back-transition was initiated with too much FW energy, about `52 m/s`
  horizontal speed, causing a large balloon/descent sequence before manual Land.

The multicopter Go-To accelerate/slow/continue behavior is still mostly PX4
point-reposition behavior, not a bridge fault. `DO_REPOSITION` is a point-hold
command; it decelerates near the clicked target. A smaller secondary issue
remains: the qtail still pitches aggressively at the start of a 5 m/s Go-To, so
the next defaults reduce horizontal acceleration and jerk while keeping the
5 m/s speed limit.

Land detection was internally consistent. The final crash was the reported
landing-gear mistake: the log shows attitude/critical failures only after the
vehicle contacted/tipped on the ground.

## Pitot and Tailsitter Sensor Frame

Do not switch sensor frames by flight mode. PX4 uses a single physical body
frame, and its actuator/control geometry is in FRD body coordinates. PX4's
fixed-wing tailsitter controllers apply an internal `+90 deg` pitch offset for
FW logic; that is controller math, not a simulator sensor-frame change.

For a pitot tube, the correct model is a physical probe axis in the aircraft
body frame. Conventional aircraft usually use `+X`. This QuadTailsitter is
different because PX4's tailsitter FW thrust maps onto the hover-body thrust
axis, so the next candidate physical pitot axis is `-Z`. That candidate should
not be used to gate transition until a dedicated pitot run validates signed
pressure against truth velocity and flow direction.

Implemented config support is intentionally small:

```ini
airspeedSource = xplane_indicated  ; xplane_indicated | disabled | body_axis
pitotAxisBody = -Z                 ; +X/-X/+Y/-Y/+Z/-Z, used by body_axis
```

`xplane_indicated` remains the QuadTailsitter default for now so negative IAS
continues to be logged as signed differential pressure. `body_axis` is available
for controlled future pitot experiments, but not enabled by the airframe
defaults.

## Implemented v3.4.33 Changes

Bridge/config:

- Added reusable `airspeedSource` and `pitotAxisBody` config fields.
- Added schema/config-editor validation for those fields.
- Added an experimental body-axis differential-pressure source without making
  it the active QuadTailsitter default.

QuadTailsitter params:

- Kept the airspeedless first-transition policy:
  `FW_USE_AIRSPD=0`, `ASPD_DO_CHECKS=0`, `SYS_HAS_NUM_ASPD=0`,
  `VT_ARSP_BLEND=0`, and `VT_ARSP_TRANS=0`.
- Reduced fixed-wing energy demand:
  `FW_THR_TRIM=0.20`, `FW_THR_MAX=0.45`, `FW_THR_SLEW_MAX=0.35`,
  and `FW_R_LIM=35`.
- Slowed and lengthened front transition:
  `VT_F_TRANS_DUR=7.0`, `VT_F_TRANS_THR=0.45`,
  `VT_TRANS_MIN_TM=6.0`, `VT_F_TR_OL_TM=10.0`, and
  `VT_TRANS_TIMEOUT=25`.
- Made FW path following conservative while airspeed is not validated:
  `NAV_LOITER_RAD=250`, `RTL_LOITER_RAD=250`, `RTL_RETURN_ALT=80`,
  `RTL_DESCEND_ALT=60`, `NPFG_PERIOD=14`, and `NPFG_ROLL_TC=0.8`.
- Smoothed MC Go-To starts without reducing the 5 m/s speed envelope:
  `MPC_XY_P=0.18`, `MPC_ACC_HOR=1.8`, `MPC_ACC_HOR_MAX=2.2`,
  `MPC_JERK_AUTO=1.2`, and `MPC_JERK_MAX=2.0`.
- Made MC landing less slow while keeping a final crawl:
  `MPC_Z_V_AUTO_DN=1.4`, `MPC_Z_VEL_MAX_DN=1.4`,
  `MPC_LAND_SPEED=0.9`, `MPC_LAND_CRWL=0.35`, and
  `LNDMC_Z_VEL_MAX=0.30`.

## Next Test

Use v3.4.33 for one controlled test:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean`.
2. Install the packaged px4xplane, XPlaneTruthCapture, and QuadTailsitter
   aircraft folders.
3. Confirm X-Plane log shows `v3.4.33`, `Config Name: QuadTailsitter`, and
   `Airspeed source=xplane_indicated pitotAxisBody=-Z`.
4. Fly MC takeoff, Hold, 3 m/s Go-To, 4-5 m/s Go-To, and 40-50 m MC Orbit.
5. Only if MC is clean, climb to at least `80 m` AGL and command one transition
   into open space.
6. If FW is stable, keep straight flight first. If you test FW loiter, use at
   least `250 m` radius.
7. Back-transition manually before RTL if FW path following looks poor.

Acceptance:

- No MC yaw/roll/pitch wobble worse than qtail10.
- Go-To can reach the 4-5 m/s range without a large pitch pulse.
- First transition does not quad-chute from low altitude.
- FW speed is materially lower than qtail10's `40-60 m/s`.
- FW loiter/orbit is judged only with the new `250 m` radius.
- Back-transition starts from a much lower-energy FW state.

## References

- PX4 tailsitter overview:
  https://docs.px4.io/v1.16/en/frames_vtol/tailsitter.html
- PX4 actuator geometry/FRD body-frame convention:
  https://docs.px4.io/main/en/config/actuators.html
- PX4 VTOL without airspeed guidance:
  https://docs.px4.io/v1.15/en/config_vtol/vtol_without_airspeed_sensor.html
- PX4 airspeed sensor guidance:
  https://docs.px4.io/v1.16/en/sensor/airspeed
