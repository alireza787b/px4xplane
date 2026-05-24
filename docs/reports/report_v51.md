# Report v51 - QuadTailsitter Design Guardrails and Pause-Safe Timestamps

Date: 2026-05-24
Package target: `v3.4.39`

## Executive Summary

The latest feedback points to two separate problems:

- The bridge had a real pause/dialog timestamp risk. Same-frame HIL messages
  could fall back to wall-clock elapsed time after X-Plane was paused or blocked
  by a dialog, which can jump PX4 timestamps far ahead of sim time. v3.4.39 fixes
  this and adds pause/resume diagnostics.
- The current QuadTailsitter flight evidence still cannot justify another broad
  aircraft retarget because qtail15 failed the PX4 parameter-sync check. X-Plane
  loaded the 5 kg aircraft, but PX4 still used stale uncanted control-allocation
  axes and old throttle/airspeed defaults.

## Design Evidence

The new design-analysis tool reports the current ACF as:

- mass: `4.99 kg` empty, `5.67 kg` max
- battery: `22.2 V`, `244 Wh`, `180 A`
- propellers: about `9.8 in`, three blades, `17,000 rpm` design speed,
  `25,000 rpm` redline
- estimated lifting area: about `0.51 m^2`
- estimated wing loading: about `96 N/m^2`
- rough stall estimate: about `11.4 m/s` at `CLmax=1.2`
- hover thrust required: about `48.9 N` total, `12.2 N` per motor
- `1.5:1` thrust-to-weight target: about `73.4 N` total

From qtail15 TruthCapture:

- X-Plane mass was stable at `4.9895 kg`
- median total prop thrust was about `47.3 N`
- max observed total prop thrust was about `60.8 N`
- median max motor throttle-used was about `0.23`
- FW true airspeed reached about `33 m/s`

This does not prove the motors are overpowered. It shows a plausible hover plant
with moderate measured thrust margin, plus an invalid FW test because PX4 was
not using the matching canted rotor geometry or current FW defaults.

## Implemented Changes

- `TimestampProvider` no longer uses absolute wall-clock elapsed time for
  same-frame calls. It now returns `last_timestamp + 1 us`, preserving monotonic
  order without jumping away from X-Plane sim time.
- The flight loop detects connected X-Plane pause/stall conditions and logs
  `[BRIDGE_PAUSE]` / `[BRIDGE_RESUME]`. It resumes sends immediately when sim
  time advances without resetting PX4 timestamps.
- Added `tools/check_px4_airframe_params.py` for ULog-vs-airframe verification.
- Added `tools/analyze_qtailsitter_design.py` for ACF and TruthCapture sizing
  summaries.
- Added offline tests for the timestamp pause fallback and the new Python tools.

## Next Test Workflow

Use v3.4.39 only after a PX4 parameter reset:

1. Pull the PX4 `px4xplane-sitl` branch containing the current
   `5021_xplane_qtailsitter`.
2. Run `make distclean` once, then `make px4_sitl_default xplane_qtailsitter`.
3. Confirm X-Plane log shows `px4xplane: Version: v3.4.39`,
   `Config Name: QuadTailsitter`, `Airspeed source=body_axis pitotAxisBody=-Z`,
   and the packaged `QuadTailsitter.acf`.
4. Do a non-flight pause/dialog check and verify `[BRIDGE_PAUSE]` and
   `[BRIDGE_RESUME]` appear without persistent PX4 EKF faults.
5. Fly MC-only first. Do not transition until MC hover, Go-To, Orbit, and land
   are clean.
6. After the run, execute:

   ```sh
   python3 tools/check_px4_airframe_params.py <ulog> config/px4_params/5021_xplane_qtailsitter \
     --param SYS_HAS_NUM_ASPD \
     --param MPC_THR_HOVER \
     --param CA_ROTOR0_AX \
     --param CA_ROTOR1_AX \
     --param CA_ROTOR2_AX \
     --param CA_ROTOR3_AX \
     --param VT_ARSP_TRANS \
     --param FW_THR_TRIM
   ```

If those values do not match, stop analysis and repeat with a clean PX4
parameter store.

## References

- PX4 Multicopter PID tuning:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- PX4 Fixed-wing rate/attitude tuning:
  https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing
- PX4 airspeed setup:
  https://docs.px4.io/main/en/config/airspeed
- PX4 tailsitter overview:
  https://docs.px4.io/v1.14/en/frames_vtol/tailsitter
- AeroVironment Quantix public spec reference:
  https://www.avinc.com/images/uploads/product_docs/Quantix_Recon_Datasheet_07122021.pdf
