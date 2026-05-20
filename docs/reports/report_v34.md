# Report v34 - evtol7 Milestone and QuadTailsitter Prep

Scope: review `/home/alireza/evtol7.zip`, close the Alia/Ehang tuning milestone,
explain the connection timeout observation, and prepare the first QuadTailsitter
test slice.

## evtol7 Evidence

- `08_44_10.ulg`: Alia, `SYS_AUTOSTART=5020`, duration about `371 s`.
- `08_51_36.ulg`: Ehang 184, `SYS_AUTOSTART=5010`, duration about `412 s`.
- TruthCapture Alia run: `36,267` frames, mean callback about `80.6 Hz`, no
  dropped rows, no sim-time resets.
- TruthCapture Ehang run: `79,658` frames, mean callback about `96.4 Hz`, no
  dropped rows, no sim-time resets.
- X-Plane `Log.txt`: px4xplane `v3.4.21`; the first Alia connect attempt timed
  out at `30 s`, then the retry connected cleanly.

## Alia Decision

Alia is accepted as a milestone baseline. The accepted run changed only one
relevant transition parameter live:

- `VT_B_TRANS_RAMP: 2.5 -> 15.0`

`VT_B_TRANS_RAMP` is the PX4 back-transition multicopter motor ramp-up window.
For this large Alia SITL model, the `15 s` value produced the accepted smoother
back-transition without disturbing the fixed-wing tune. v3.4.22 adopts it and
does not change the accepted TECS, NPFG, bank limit, front-transition, or
no-default-brake policy.

The remaining late disarm is a land-detector timing issue, not an in-flight
estimator or control problem. In the accepted Alia ULog, `landed=true` toggled
false briefly after touchdown, then PX4 disarmed 2 s after the final stable
landed state. v3.4.22 sets `COM_DISARM_LAND=1.5` for SITL demo ergonomics.

## Ehang Decision

The Ehang ULog includes the user's live tuning sequence. The final stable values
before RTL were:

- `MC_ROLLRATE_K=3.0`
- `MC_PITCHRATE_K=2.05`
- `MC_ROLL_P=1.0`
- `MC_PITCH_P=1.0`

Measured attitude tracking improved after those changes: final orbit/RTL
segments had roll error RMS around `0.9-1.3 deg` and pitch error RMS around
`1.6-2.1 deg`, with no in-flight estimator warnings. v3.4.22 makes these the
Ehang defaults and also sets `COM_DISARM_LAND=1.5`.

## Connection Timeout

The first Alia connection timeout was not the v3.4.20 regression. v3.4.21 kept
the recovered blocking connected stream, and the retry connected immediately.
The likely mechanism is PX4's simulator TCP retry cadence: if PX4 and the
plugin miss each other during startup, the plugin can time out before the next
client attempt lands. v3.4.22 extends the visible wait window from `30 s` to
`60 s`; a manual retry remains safe and does not change airframe state.

## QuadTailsitter Prep

The uploaded QuadTailsitter source `.acf` was the source model for the QuadTailsitter
package. It is a four-motor tailsitter with no configured control surfaces. The
file shows:

- empty mass about `5 lb` (`2.27 kg`), max mass about `6 lb`
- four direct electric motors
- motor offsets near `+/-1.4 ft` lateral and `+/-0.72 ft` longitudinal
- no active aileron/elevator control surfaces in the inspected wing entries

The previous `5021_xplane_qtailsitter` overclaimed readiness and used a very
fast first-test envelope. v3.4.22 replaces it with a conservative first-test
file based on PX4's official quadtailsitter SITL pattern:

- `MAV_TYPE=20`, `CA_AIRFRAME=4`, `CA_SV_CS_COUNT=0`
- `MC_AIRMODE=2`, `MC_ROLL_P=3.0`, `MC_PITCH_P=3.0`
- `FW_AIRSPD_MIN=14`, `FW_AIRSPD_TRIM=18`, `FW_AIRSPD_MAX=24`
- `VT_ARSP_TRANS=15`, `VT_F_TRANS_DUR=1.5`, `VT_B_TRANS_DUR=5`
- `VT_FW_DIFTHR_EN=7`
- `NAV_LOITER_RAD=120`, `RTL_LOITER_RAD=120`
- px4xplane baro/GPS/IMU settings aligned with the Alia/Ehang baseline

## References Used

- PX4 Tailsitter VTOL docs: https://docs.px4.io/v1.14/en/frames_vtol/tailsitter.html
- PX4 VTOL configuration index: https://docs.px4.io/main/en/config_vtol/
- PX4 parameter reference for `VT_B_TRANS_RAMP` and back-transition parameters:
  https://docs.px4.io/v1.12/en/advanced_config/parameter_reference
- PX4 land-detector docs for `COM_DISARM_LAND`:
  https://docs.px4.io/main/en/advanced_config/land_detector.html
- AeroVironment Quantix Recon datasheet for scale context only:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf

## Next Test

Use v3.4.22 for one QuadTailsitter first test:

1. Set px4xplane `config_name = QuadTailsitter`.
2. Run `make px4_sitl_default xplane_qtailsitter`.
3. Confirm `SYS_AUTOSTART=5021`.
4. Start TruthCapture before connecting.
5. Take off to `50-60 m`, hover, transition, hold or wide loiter, RTL,
   back-transition, land, and wait after disarm.

Abort early if hover axes are wrong, airspeed remains zero, or fixed-wing mode
cannot hold altitude. The first QuadTailsitter log should be treated as a motor-order,
axis-sign, and conservative-envelope validation, not a final performance demo.
