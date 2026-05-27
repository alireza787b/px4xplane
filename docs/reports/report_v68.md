# Report v68 - Cessna3 Surface Smoothing and Flare Recovery

Date: 2026-05-27

## Evidence Reviewed

- Bundle: `/home/alireza/cessna3.zip`
- X-Plane log: px4xplane `v3.4.55`, active config `Cessna172`
- TruthCapture: `76,302` frames, `0` dropped rows, `0` sim-time resets,
  roughly `96 Hz` mean callback rate
- ULog: PX4 branch `px4xplane-sitl`, commit `74f7ea3`, no dropouts

## Findings

1. Control-surface jerk was real, not only visual.

   `actuator_outputs[0]` and `[1]` repeatedly reached `1000..2000 us`, and
   `actuator_servos.control[0/1]` repeatedly reached `-1..+1`. The Cessna
   ailerons therefore were being commanded close to their configured limits.
   The older `FW_RR_FF=4.8` value was not consistent with PX4 fixed-wing tuning
   guidance, which recommends starting roll-rate feed-forward near `0.4` and
   increasing only until the roll response tracks cleanly.

2. X-Plane rendering made the saturated commands look even more discrete.

   PX4 HIL actuator packets arrived near `10 Hz`, while the simulator frame
   loop was around `96 Hz`. Without interpolation, X-Plane holds each actuator
   command for several rendered frames.

3. The landing-flap mapping did not work on the loaded Laminar C172.

   X-Plane reported both `sim/flightmodel/controls/wing1l_flap1def` and
   `sim/flightmodel/controls/wing1r_flap1def` missing. PX4 produced nonzero
   flap outputs, but the aircraft stayed at `flap_ratio=0.0`, so the test was
   effectively a no-flap landing.

4. Flare was entered, but touchdown looked flat.

   PX4 logged `Landing, flaring`, but the missing flaps and
   `FW_LND_TD_TIME=5` touchdown clamp made the flare look short/flat. The next
   test should judge flare only after confirming the flap ratio is nonzero.

## Implemented Changes

- Added generic, opt-in bridge smoothing:
  `actuatorSmoothingTimeConstantSec`.
  - Default remains `0.0`, so existing airframes keep direct pass-through.
  - Cessna uses `0.08 s` to smooth low-rate HIL actuator packets over the
    faster X-Plane frame loop.

- Fixed Cessna flap outputs:
  - `channel6` and `channel7` now write
    `sim/cockpit2/controls/flap_ratio`.
  - PX4 left/right flap outputs are equal in this Cessna airframe, so duplicate
    writes to the shared flap command are intentional.

- Reduced Cessna roll activity:
  - `FW_RR_FF: 4.8 -> 0.50`
  - `FW_RR_P: 0.47 -> 0.30`
  - `FW_RR_I: 0.13 -> 0.08`
  - `FW_RR_D: 0.085 -> 0.03`
  - `FW_R_RMAX: 20 -> 15 deg/s`
  - `FW_R_TC: 0.9 -> 1.0`
  - `FW_R_LIM: 35 -> 30 deg`

- Adjusted the Cessna landing baseline:
  - `FW_LND_AIRSPD: 31 -> 33 m/s`
  - `FW_LND_FL_PMIN: 5 -> 8 deg`
  - `FW_LND_FL_SINK: 0.25 -> 0.30 m/s`
  - `FW_LND_TD_TIME: 5 -> -1`

## Next Test Checks

1. Run `make px4_sitl_default distclean` before testing.
2. Confirm X-Plane `Log.txt` says `px4xplane: Version: v3.4.56`.
3. Confirm there are no Cessna flap dataref validation errors.
4. Confirm the log says actuator smoothing is enabled for Cessna.
5. In TruthCapture, check that `sim/cockpit2/controls/flap_ratio` becomes
   nonzero on approach.
6. In the ULog, verify aileron outputs no longer sit at min/max for long
   cruise/landing periods.
7. Judge landing energy by stabilized approach, visible roundout, no bounce,
   and controlled rollout.

## References

- PX4 fixed-wing rate/attitude tuning guide: roll feed-forward starts near
  `0.4`, then P is increased only until wobble/twitch appears and backed off.
- PX4 fixed-wing landing documentation and local PX4 parameter source:
  `FW_LND_FL_PMIN` controls flare minimum pitch, and `FW_LND_TD_TIME=-1`
  disables touchdown-time clamping.
