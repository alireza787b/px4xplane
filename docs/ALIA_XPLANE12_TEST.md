# Alia 250 X-Plane 12 Test Card

Use the built-in Alia with the matching `Alia250` plugin configuration and PX4
`xplane_alia250` target (`SYS_AUTOSTART=5020`). This is a Standard VTOL with four
lift motors, a pusher, and control surfaces, not a tailsitter.

## Setup

1. Follow the [normal setup](../README.md) or [guided launcher](guided-launcher.md).
   Use the current released plugin and matching `config.ini`. Restart X-Plane
   after replacing a plugin binary.
2. Load Alia on a clear runway/apron. For the first comparison use calm weather,
   unchanged loading/CG, and repeatable rendering/physics settings. Record the
   airport, X-Plane version, flight-model calculations per frame, actual mass
   and CG. Do not substitute brochure weight for loaded mass.
3. Select **Plugins > PX4 X-Plane > Airframes > Alia250**, then **Connect to SITL**.
   Start the matching PX4 target:

   ```bash
   make px4_sitl_default xplane_alia250
   ```

   Either side can start first. If the initial build exceeds the connection
   wait, click **Connect to SITL** again when PX4 is waiting for the simulator.
   For Windows/WSL networking and QGC, use the linked setup instructions.
4. Check `ver all`, `param show SYS_AUTOSTART`, and `mavlink status` in the PX4
   console. Confirm the QGC vehicle and a receiving link before relying on QGC
   commands. Outgoing UDP traffic alone does not establish reception.

Normal guided setup uses official PX4 without pending fixes. `--validation` is
a separate TECS proposal test, not Alia tuning: inspect its printed stack. Previously tested
GSF, Standard VTOL, and tailsitter guards must not be blindly reapplied.

The November 8 [accepted Alia simulator profile](alia250-profile.md) is recorded
separately, with exact parameters, source, results and remaining limits.
Normal official commands do not apply that profile.

## Test Order

### MC Baseline

- Check all four lift motors and the pusher against the loaded model's engine
  order. Inspect surface directions before flight. The current elevator mapping
  includes all four `hstab1/2_elv1/2def` datarefs; check which exist and move in
  the actual model, rather than assuming the mapping establishes authority.
- Take off to 30 m, hover for 30 s, then command two modest GoTo moves.
- Land and wait for automatic disarm. Retain any post-landing warnings. Do not
  increase estimator limits or force arm to obtain a passing run.

### Transition And Cruise

Proceed only after MC is controllable. Use an open area and about 200 m AGL for
the initial simulator test, not as a certified operating limit.

- Transition on a long outbound segment, avoiding tight path capture during
  handoff. Record airspeed, pitch, thrust, altitude and transition state.
- Hold a straight leg and broad loiter, then repeat in the opposite direction
  at the same altitude. The current 55 m/s trim, 20 degree bank limit and 2000 m
  loiter are baseline settings, not a validated flight envelope.
- Back-transition, allow hover to settle, then Land. For RTL record actual
  navigation and VTOL states, rather than inferring them from the map.
- Wait 10-15 s after disarm before `shutdown`. If landing stalls, keep logging
  for 30 s and capture controller setpoints/mode state before ending the run.

Console equivalents are `commander takeoff`, `commander transition`,
`commander mode auto:loiter`, and `commander land`. A repeated transition command
changes the requested VTOL mode; do not repeat without checking state.
`commander mode hold` and `commander mode land` are not valid equivalents.

## Evidence

Start XPlaneTruthCapture after loading Alia and before connecting PX4. Confirm
the recording is active and `frames.csv` grows. Preserve its manifest and dataref
catalog; a leftover recording is not evidence of this flight.

For every comparison retain:

- PX4 ULog, exact source commit, startup/console output, effective parameters and
  in-flight changes. Use fresh or explicitly reset **selected** SITL state.
- Loaded `.acf`, `config.ini`, plugin version/binary identity and X-Plane
  `Log.txt`. Reference copies do not establish what the simulator loaded.
- TruthCapture for the same flight: position/velocity, attitude/rates, physical
  actuator responses and available forces/moments, mass and CG. Report missing
  datarefs; do not treat them as zero.
- Scenario, weather, loading, physics/render settings and QGC connection state.

Plugin-package `px4_airframes` files are references only. PX4 loads its own file
under `ROMFS/px4fmu_common/init.d-posix/airframes/5020_xplane_alia250`.
Names/defaults change upstream: compare against the **exact tested revision**,
not a duplicated parameter list in this document.

## Analysis And Acceptance

Tune inner rate/attitude loops before position/guidance and TECS, then assess
transition handoffs. Compare setpoints to measured rates, allocation to physical
actuator response, and EKF estimates to independent truth before assigning a
fault to gains, estimation or aircraft dynamics. PX4 performance weights do not
change the X-Plane model's mass.

For a stopped descent trace command acceptance, navigation state, vertical
velocity demand, thrust and actual motion separately. Oscillation alone does
not prove PX4 rejected Land. Keep estimator warnings and low-FPS failures as
evidence; do not hide them by relaxing arming limits.

Accept a stage only with demonstrated tracking, actuator margin, no unexplained
estimator rejection/reset, controlled transitions and completed landing/disarm.
A normal process exit or one flight does not prove the full speed, mass, wind,
CG or frame-rate envelope.

Initial repository checks:

```bash
python3 tools/analyze_ulog_estimator.py /path/to/log.ulg --json
python3 tools/check_px4_airframe_params.py /path/to/log.ulg /path/to/tested/5020_xplane_alia250 --json
python3 tools/replay_truth_capture.py /path/to/capture.zip --output /tmp/alia_bridge_replay.csv
```

Replay checks deterministic bridge signs, units and timing; it does not simulate
aircraft aerodynamics or establish controller stability.

## References

- [PX4 X-Plane setup](https://docs.px4.io/main/en/sim_xplane/)
- [Standard VTOL tuning](https://docs.px4.io/main/en/config_vtol/vtol_quad_configuration)
- [MC PID tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter)
- [FW attitude tuning](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing)
- [FW altitude/position tuning](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing)
