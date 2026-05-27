# Cessna 172 X-Plane 12 Test Card

Use this card for the first validation of the refreshed Cessna 172 runway
mission baseline.

## Package And PX4 State

1. Install the px4xplane package and keep:

   ```ini
   config_name = Alia250
   ```

   for normal demo startup. Before the Cessna run, use the config editor or
   direct INI edit to set:

   ```ini
   config_name = Cessna172
   ```

2. Pull the PX4 branch containing `5001_xplane_cessna172` and run one PX4
   parameter reset after replacing the airframe file:

   ```bash
   make px4_sitl_default distclean
   ```

3. Launch:

   ```bash
   make px4_sitl_default xplane_cessna172
   ```

4. Use the X-Plane Cessna 172 on a paved runway. Align the aircraft on the
   runway heading before arming. Release parking brake before mission start.

5. Start XPlaneTruthCapture before connecting PX4.

## Mission Shape

Use a simple, conservative fixed-wing mission:

1. Runway takeoff to `100 m` AGL.
2. One straight outbound waypoint at least `900 m` from the runway.
3. One gentle turn waypoint or loiter with radius around `600 m`.
4. Fixed-wing landing pattern back to the same runway.

Avoid judging the first test with a very tight loiter or short waypoint spacing.
The Cessna is a manned fixed-wing aircraft, not an agile UAV.

## Expected Values

PX4 airframe sanity values:

```text
SYS_AUTOSTART = 5001
FW_USE_AIRSPD = 1
ASPD_DO_CHECKS = 1
ASPD_FALLBACK = 1
SYS_HAS_NUM_ASPD = 0
FW_AIRSPD_MIN = 30
FW_AIRSPD_TRIM = 50
FW_AIRSPD_MAX = 75
FW_TKO_AIRSPD = 38
RWTO_TKOFF = 1
RWTO_ROT_AIRSPD = 28
FW_W_EN = 1
FW_LND_AIRSPD = 36
FW_LND_ANG = 5
NAV_ACC_RAD = 180
NAV_LOITER_RAD = 600
```

Bridge sanity values:

```text
Config Name: Cessna172
Active Airframe: Cessna172
Airspeed source: xplane_indicated
Camera Views: Forward, Down, Chase
```

## Acceptance Criteria

- Runway takeoff accelerates, rotates, and climbs without wingstrike or
  immediate heading divergence.
- No in-flight EKF, baro stale, accelerometer bias, or airspeed selector
  warnings after sensors settle.
- TECS holds altitude without phugoid-like throttle/pitch hunting in straight
  cruise.
- NPFG turns are broad and coordinated; no repeated path overshoot around the
  `600 m` loiter.
- Mission landing follows the final approach path, flares near the runway, and
  disarms after rollout.

## Log Bundle

Return:

- PX4 ULog
- X-Plane `Log.txt`
- PX4 terminal log
- XPlaneTruthCapture folder or zip
- Notes on runway heading, wind, mission waypoint spacing, and any QGC warning

Run the static parameter check before interpreting flight behavior:

```bash
python3 tools/check_px4_airframe_params.py /path/to/cessna.ulg \
  --airframe config/px4_params/5001_xplane_cessna172
```

If the ULog does not contain the values above, reset PX4 SITL parameters and
rerun before tuning.
