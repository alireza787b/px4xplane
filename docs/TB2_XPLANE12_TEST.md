# TB2 X-Plane 12 Test Card

Use this card for the first refreshed Bayraktar TB2-style fixed-wing validation.

## Package Check

1. Copy the packaged `px4xplane` plugin into `X-Plane 12/Resources/plugins/`.
2. Copy the `Bayraktar_tb2` aircraft folder into an X-Plane aircraft folder.
3. Confirm X-Plane `Log.txt` contains:

```text
px4xplane: Version: v3.4.58
px4xplane: Loaded configuration: TB2
px4xplane: Actuator command smoothing enabled (tau 0.040s, channels 0,1,2,3)
```

4. Confirm the config editor shows `TB2` as the selected airframe.

## PX4 Check

Use the PX4 fork branch with `5002_xplane_tb2` installed in:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/5002_xplane_tb2
```

Run a clean build after airframe changes:

```bash
make px4_sitl_default distclean
make px4_sitl_default xplane_tb2
```

Before judging the flight, check the ULog initial parameters with:

```bash
python3 tools/check_px4_airframe_params.py <ulog> px4_airframes/5002_xplane_tb2
```

Important expected parameters:

```text
SYS_AUTOSTART=5002
PWM_MAIN_FUNC6=440
PWM_MAIN_FUNC7=205
PWM_MAIN_FUNC8=206
FW_W_EN=1
ASPD_DO_CHECKS=1
ASPD_FALLBACK=1
SYS_HAS_NUM_ASPD=0
EKF2_ABL_LIM=0.8
IMU_GYRO_RATEMAX=200
IMU_INTEG_RATE=200
```

## Flight Script

1. Start X-Plane with the Bayraktar TB2 aircraft on a long runway.
2. Start PX4 with `xplane_tb2`.
3. Wait for `Ready for takeoff` and verify QGC sees a valid airspeed.
4. Create a mission with:
   - runway takeoff,
   - first waypoint straight ahead at least 1.5 km from runway end,
   - cruise waypoints at 150 m AGL or higher,
   - a loiter radius around 700 m,
   - runway or fixed-wing landing.
5. Run the mission without joystick intervention unless safety requires aborting.
6. After landing/disarm, wait 10 seconds before stopping PX4.

## What To Record

Send back:

- PX4 `.ulg`
- PX4 terminal log
- X-Plane `Log.txt`
- XPlaneTruthCapture folder or zip
- Any visible observations about:
  - runway centerline tracking,
  - surface smoothness,
  - flap deployment,
  - airspeed target tracking,
  - loiter/orbit capture,
  - flare and rollout.

## Expected First-Pass Behavior

This is a first refreshed TB2 baseline, not yet a closed airframe. The target is
safe autonomous runway takeoff, cruise, loiter, and landing with no EKF warnings.
Fine tuning should be based on returned ULog evidence.
