# Report v69 - Cessna4 Path Authority, Flaps, and Brake Review

Date: 2026-05-27  
Package target: px4xplane v3.4.57  
Primary evidence: `/home/alireza/cessna4.zip`

## What Failed

- `cessna4` did not fail because of low FPS. TruthCapture recorded `72,145`
  frames with `0` drops and `0` sim-time resets, around `95 Hz` mean callback
  rate.
- Takeoff drifted left even though PX4 commanded strong wheel steering. The
  bridge had smoothed every actuator channel in v3.4.56, so the nosewheel was
  delayed along with the control surfaces.
- Airborne path tracking regressed because the v3.4.56 Cessna roll tune was too
  soft. Aileron motion was less jerky, but the aircraft started roll correction
  late and oscillated around the path.
- The flap handle moved, but actual flaps did not deploy. X-Plane 12 marks the
  old shared flap datarefs as replaced; TruthCapture showed the old handle path
  stayed at `0.0`.

## Fixes Applied

- Added generic `actuatorSmoothingChannels` support. Cessna now smooths only
  channels `0,1,2,4` with `0.04 s`; throttle, nosewheel, and flaps are direct.
- Restored Cessna roll authority near the last accepted test without returning
  to the earlier saturated tune:
  `FW_RR_FF=3.2`, `FW_RR_P=0.42`, `FW_RR_D=0.08`, `FW_R_RMAX=20`.
- Updated Cessna flaps to current X-Plane 12 datarefs:
  `sim/cockpit2/controls/flap_handle_request_ratio`,
  `sim/flightmodel/controls/wing1l_fla1def`, and
  `sim/flightmodel/controls/wing1r_fla1def`.
- Corrected the same flap spelling for TB2 so the next fixed-wing airframe does
  not inherit the stale `flap1def` naming.
- Updated XPlaneTruthCapture defaults to log the current flap/brake datarefs,
  so future tests can prove whether physical flaps and brakes moved.

## Brake Decision

PX4 exposes `Landing_Gear_Wheel` for runway steering in this branch, and that
is already mapped. It does not expose a native fixed-wing wheel-brake output
function in the actuator function set we are using. X-Plane has writable toe
brake datarefs, but adding hidden bridge-side autobrake would be simulator-only
behavior and could mask PX4 behavior. This release does not add automatic
wheel braking.

## Next Test

1. Install v3.4.57.
2. Pull the PX4 branch and run:

   ```bash
   make px4_sitl_default distclean
   ```

3. Start XPlaneTruthCapture.
4. Launch `make px4_sitl_default xplane_cessna172`.
5. Verify X-Plane `Log.txt` contains:

   ```text
   px4xplane: Version: v3.4.57
   Actuator command smoothing enabled (tau 0.040s, channels 0,1,2,4)
   Parsed channel 5
   Parsed channel 6
   Parsed channel 7
   ```

6. Run the same Cessna mission. Expected result: centerline takeoff recovers,
   airborne path capture returns, control surfaces remain less visually stepped
   than v3.4.55, and landing flaps visibly deploy.

