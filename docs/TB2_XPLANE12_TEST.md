# TB2 X-Plane 12 Test Card

Use this card for the refreshed Bayraktar TB2-style fixed-wing validation.

## Package Check

1. Copy the packaged `px4xplane` plugin into `X-Plane 12/Resources/plugins/`.
2. Copy the `Bayraktar_tb2` aircraft folder into an X-Plane aircraft folder.
3. Confirm X-Plane `Log.txt` contains:

```text
px4xplane: Version: v4.1.1
px4xplane: Loaded configuration: TB2
px4xplane: Actuator command smoothing enabled (tau 0.040s, channels 0,1,2,3)
```

4. Confirm the HUD or `Log.txt` does not show a config warning after the
   Bayraktar `.acf` is loaded. The current package defers the match check while
   X-Plane is still reporting only the simulator root path during startup.
5. Confirm the config editor shows `TB2` as the selected airframe.

## PX4 Check

Run PX4 with the validation launcher and select `xplane_tb2`:

```bash
px4xplane --validation --reset-config
```

The printed validation stack should show official PX4 plus the pending EKF-GSF,
Standard VTOL, TECS, and tailsitter PR guards unless you are deliberately
testing `--official`.

The setup script now resets the saved PX4 SITL parameter store automatically
when the selected X-Plane airframe or its default airframe file fingerprint
changes. That keeps stale `parameters.bson` values from masking the current
defaults.

For a fully manual run, still use a clean build after airframe changes:

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
CAL_ACC0_XOFF=0.0
CAL_ACC0_YOFF=0.0
CAL_ACC0_ZOFF=0.0
IMU_GYRO_RATEMAX=200
IMU_INTEG_RATE=200
SENS_GPS0_DELAY=10
SENS_GPS1_DELAY=10
EKF2_MAG_TYPE=1
EKF2_MULTI_IMU=1
SENS_IMU_MODE=0
FW_AIRSPD_TRIM=36.0
FW_LND_AIRSPD=28.0
FW_T_ALT_TC=8.0
FW_T_HRATE_FF=0.15
FW_T_SPDWEIGHT=1.0
FW_THR_TRIM=0.32
FW_LND_THRTC_SC=0.7
FW_FLAPS_LND_SCL=1.0
LNDFW_AIRSPD_MAX=12.0
LNDFW_VEL_XY_MAX=6.0
LNDFW_TRIG_TIME=2.0
COM_DISARM_LAND=4.0
```

Keep the accelerometer offsets neutral. The bridge already publishes
attitude-consistent HIL accelerometer data; nonzero offsets can corrupt
`sensor_combined` and trigger EKF2 accelerometer-bias checks while parked.

Abort the test if the ULog still shows stale values such as
`EKF2_ABL_LIM=2.0`, `SENS_GPS0_DELAY=0`, `EKF2_MULTI_IMU=3`, or
`IMU_INTEG_RATE=250`. Run `make px4_sitl_default distclean`, then delete any
saved SITL parameter store if stale values remain:

```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson \
      build/px4_sitl_default/rootfs/parameters_backup.bson
```

Do not judge estimator warnings or landing behavior until these values match the
airframe defaults.

## Flight Script

1. Start X-Plane with the Bayraktar TB2 aircraft on a long runway.
2. Start PX4 with `xplane_tb2`.
3. Wait for `Ready for takeoff` and verify QGC sees a valid airspeed.
4. Create a mission with:
   - runway takeoff,
   - first waypoint straight ahead at least 1.5 km from runway end,
   - cruise waypoints at 150 m AGL or higher,
   - a loiter radius around 700 m,
   - a runway or fixed-wing landing with a long final approach. If QGC/PX4
     reports an infeasible landing slope, increase approach distance or lower
     the landing entrance altitude instead of making the slope steeper.
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

## Expected Closure Behavior

The target is safe autonomous runway takeoff, cruise, loiter, and landing with
no EKF warnings, controlled centerline tracking, stable flap deployment, and a
landing approach that converges toward the configured `28 m/s` airspeed without
the previous high-energy final. Rollout should stay armed and steered longer
than the earlier high-energy landing baseline, then disarm after the aircraft has slowed to a low taxi speed.
