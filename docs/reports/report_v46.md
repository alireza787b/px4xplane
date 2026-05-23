# Report v46 - QuadTailsitter qtail11 Body-Axis Pitot and Path Recovery

Date: 2026-05-23

## Scope

This report reviews `/home/alireza/qtail11.zip`, answers the pitot, mass,
motor-cant, lift, battery, and path-following questions, and defines the next
QuadTailsitter test package.

## Log Integrity

- Zip test: OK.
- PX4 ULog: `02_39_18.ulg`, about `558.4 s`.
- XPlaneTruthCapture: `45,956` frames, `0` dropped rows, mean callback rate
  about `77.3 Hz`, max frame period about `50 ms`.
- X-Plane loaded px4xplane `v3.4.33` and
  `Aircraft/QuadTailsitter/QuadTailsitter.acf`.
- The active bridge config was:
  `airspeedSource=xplane_indicated` and `pitotAxisBody=-Z`.
- The active PX4 defaults were still airspeedless:
  `FW_USE_AIRSPD=0`, `SYS_HAS_NUM_ASPD=0`, `VT_ARSP_TRANS=0`.

## Direct Answers

qtail11 did not use the modeled body-axis pitot for PX4 transition or TECS.
PX4 was still airspeedless. The QGC negative airspeed came from the diagnostic
X-Plane IAS differential-pressure path, not from an airspeed sensor that PX4
trusted for control.

The body-axis pitot calculation itself is now validated from truth data. In
fixed-wing attitude, projecting local velocity onto `pitotAxisBody=-Z` gives a
positive median airspeed around `39.5 m/s`, close to X-Plane true airspeed.
The native X-Plane indicated-airspeed dataref was negative in the same segment,
with median around `-77 kt`, which explains the negative QGC display.

Do not change the current aircraft to `5 kg` in this slice. The ACF value
`acf/_m_empty=5` is in pounds, which gives the observed `2.27 kg`. That is
consistent with public Quantix-class specifications. qtail11 truth showed the
modeled total thrust peaks are not sized for a `5 kg` hover without a larger
motor/prop/wing redesign.

Do not add static motor cant in this slice. The log does not show torque
authority as the root FW path issue. PX4 requested FW bank, the tailsitter FW
roll setpoint reached `+/-35 deg`, and the transformed actual roll followed it
reasonably. If motor cant is added later, PX4 `CA_ROTOR*_AX/AY/AZ` must be
updated with the same physical rotor-axis vectors; changing only the ACF would
make the simulator and PX4 allocator disagree.

Battery current/voltage modeling is feasible, but not as a quick qtail tuning
fix. px4xplane can estimate electrical power from motor commands and X-Plane
engine power/torque data, then send MAVLink `BATTERY_STATUS` or related power
messages. That should be a separate battery-model slice with configurable
cell count, capacity, internal resistance, and voltage sag, because PX4 SITL
currently does not receive that from this bridge.

## qtail11 Findings

The startup land-detector warning was caused by PX4's own multicopter land
detector constraint:

`LNDMC_Z_VEL_MAX <= min(MPC_LAND_CRWL, MPC_LAND_SPEED) / 1.2`

With `MPC_LAND_CRWL=0.35`, the upper threshold is about `0.292`, so PX4
corrected `LNDMC_Z_VEL_MAX=0.30` to `0.292`. v3.4.34 raises
`MPC_LAND_CRWL` to `0.36` so `LNDMC_Z_VEL_MAX=0.30` is valid at startup.

The fixed-wing path error was not primarily a missing-bank problem. In the
tailsitter fixed-wing frame:

- FW roll setpoint reached `+/-35 deg`.
- Actual transformed roll had p95 about `34.7 deg`.
- Roll setpoint and required bank were strongly correlated.

The path was large because the vehicle was flying about `42 m/s` while trying
to loiter with a `250 m` radius. The fitted pre-RTL truth path radius was about
`440 m`. At `40-50 m/s`, a comfort-level `12-15 deg` bank implies radii of
hundreds of meters. v3.4.34 uses the now-validated pitot to target a lower
Quantix-class FW speed and moves the default FW loiter radius to `300 m`.

The back-transition/RTL miss was a high-energy consequence of the same speed
problem. Back-transition started around `40-60 m/s`; after returning to MC
mode, the vehicle ballooned and spent a long time recovering. Lowering the FW
speed target is the primary fix; v3.4.34 also slightly lengthens and softens
back-transition deceleration.

The wings are producing lift. During the FW segment, the vehicle held altitude
for long periods with TECS throttle around `0.18` and vertical velocity near
zero. With `2.27 kg` mass, the wing must be carrying roughly `22 N` in level
flight, higher in bank. At the observed `40+ m/s`, this implies low lift
coefficient, so the vehicle is not lift-starved; it is energy/speed/radius
mismatched.

## Implemented v3.4.34 Changes

Bridge/config:

- Changed QuadTailsitter default to:
  `airspeedSource=body_axis` and `pitotAxisBody=-Z`.
- Reused the configured airspeed source for `HIL_STATE_QUATERNION` unsigned
  airspeed fields so QGC no longer receives zero indicated airspeed there while
  `HIL_SENSOR.diff_pressure` is valid.
- Made the body-axis pitot projection wind-aware by subtracting X-Plane's local
  wind vector before transforming velocity into body axes. Calm-weather qtail11
  evidence still drives this slice, but the implementation is no longer only a
  ground-speed projection.

QuadTailsitter PX4 params:

- Enabled airspeed feedback:
  `FW_USE_AIRSPD=1`, `SYS_HAS_NUM_ASPD=1`, `ASPD_DO_CHECKS=0`.
- Set first airspeed-gated transition targets:
  `VT_ARSP_BLEND=14`, `VT_ARSP_TRANS=20`, `VT_F_TRANS_DUR=8.0`,
  `VT_F_TR_OL_TM=12.0`, and `VT_TRANS_TIMEOUT=30`.
- Moved FW speed/energy to a Quantix-class first target:
  `FW_AIRSPD_MIN=16`, `FW_AIRSPD_TRIM=24`, `FW_AIRSPD_MAX=35`,
  `FW_THR_TRIM=0.12`, `FW_THR_MIN=0.03`, and `FW_THR_SLEW_MAX=0.30`.
- Softened back-transition:
  `VT_B_TRANS_DUR=8.0`, `VT_B_DEC_MSS=1.2`, and `VT_B_TRANS_RAMP=6.0`.
- Enlarged FW loiter:
  `NAV_LOITER_RAD=300`, `RTL_LOITER_RAD=300`.
- Removed the land-detector startup correction:
  `MPC_LAND_CRWL=0.36` with `LNDMC_Z_VEL_MAX=0.30`.

No aircraft mass, motor-power, rotor-position, or motor-cant changes were made
in this slice.

## Next Test

Use v3.4.34 for one controlled test:

1. Pull the PX4 `px4xplane-sitl` branch and run `make distclean`.
2. Install the packaged px4xplane, XPlaneTruthCapture, and QuadTailsitter
   aircraft folders.
3. Confirm X-Plane log shows `v3.4.34`, `Config Name: QuadTailsitter`, and
   `Airspeed source=body_axis pitotAxisBody=-Z`.
4. Confirm the ULog starts with `FW_USE_AIRSPD=1`, `SYS_HAS_NUM_ASPD=1`,
   `VT_ARSP_TRANS=20`, and `FW_AIRSPD_TRIM=24`.
5. Fly MC takeoff, Hold, 3 m/s Go-To, 4-5 m/s Go-To, and 40-50 m MC Orbit.
6. If MC is clean, climb to at least `80 m` AGL and command one transition.
7. If FW is stable, keep straight flight first. If you test FW loiter, use at
   least `300 m` radius.
8. Use RTL or manual back-transition only after observing whether FW airspeed
   settles near the new target instead of running away to `40-50 m/s`.

Acceptance for the next log:

- No land-detector startup correction.
- QGC/ULog airspeed is positive in FW.
- Transition waits for airspeed instead of switching at near-zero FW speed.
- FW true/validated airspeed trends toward the `20-30 m/s` range.
- FW loiter is judged against `300 m` radius.
- Back-transition does not balloon hundreds of meters above the target.

## References

- PX4 VTOL configuration:
  https://docs.px4.io/main/en/config_vtol/
- PX4 QuadPlane transition tuning:
  https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration
- PX4 VTOL without airspeed sensor:
  https://docs.px4.io/v1.15/en/config_vtol/vtol_without_airspeed_sensor.html
- PX4 tailsitter overview:
  https://docs.px4.io/v1.16/en/frames_vtol/tailsitter.html
- Quantix Recon public specifications:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf
