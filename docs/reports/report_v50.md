# Report v50 - QuadTailsitter qtail15 Parameter Sync and MC Pitch Fix

Date: 2026-05-24
Package target: `v3.4.38`

## Input Evidence

- `/home/alireza/qtail15.zip`
- ULog: `/tmp/qtail15/03_18_29.ulg`
- TruthCapture: `/tmp/qtail15/20260524-031749Z`
- X-Plane log: `/tmp/qtail15/Log.txt`

## Executive Summary

qtail15 was not a valid test of the intended v3.4.37 PX4 airframe defaults.
X-Plane loaded the new 5 kg aircraft and px4xplane v3.4.37, but PX4 kept stale
saved parameters from an older QuadTailsitter setup. The ULog showed:

- `SYS_HAS_NUM_ASPD=1`, not the expected `0`
- `MPC_THR_HOVER=0.27`, not the measured v3.4.38 target `0.22`
- `VT_ARSP_TRANS=18`, not the expected `24`
- all `CA_ROTOR*_AX=0`, `CA_ROTOR*_AY=0`, `CA_ROTOR*_AZ=-1`, so PX4 did not
  know about the 5 degree motor cant
- `FW_THR_TRIM=0.05` and `FW_THR_MAX=0.25`, not the v3.4.37 values

The X-Plane plant itself did load correctly:

- `sim/flightmodel/weight/m_total = 4.9895 kg`
- hover used roughly `0.19-0.21` throttle
- total hover thrust was about `48-50 N`, matching the 5 kg weight

The immediate fix is therefore not another broad plant rewrite. The next test
must force PX4 to load the current airframe defaults, and the MC pitch gain must
be reduced based on the qtail15 live-tuning evidence.

## Airspeed Warning

The old `Preflight Fail: Airspeed selector module down` message is explained by
stale PX4 params in this run. qtail15 still had `SYS_HAS_NUM_ASPD=1`, so PX4
treated the virtual X-Plane pitot as a required prearm hardware sensor. Current
airframe defaults keep airspeed for control (`FW_USE_AIRSPD=1`) but set
`SYS_HAS_NUM_ASPD=0`, so the virtual pitot is not a prearm hardware requirement.

## MC Pitch Oscillation

The user changed three parameters during the flight:

- `98.908 s`: `MC_PITCHRATE_D=0.003`
- `109.072 s`: `MC_PITCHRATE_K=0.8`
- `122.822 s`: `MC_PITCH_P=0.5`

Measured ULog response:

- before those changes, pitch-rate RMS was about `0.79 rad/s`, and pitch angle
  peak-to-peak was about `87 deg`
- increasing pitch-rate D did not remove the oscillation
- reducing pitch-rate K alone did not remove the oscillation
- after `MC_PITCH_P=0.5`, pitch-rate RMS dropped to about `0.11 rad/s`
- in the later MC orbit/hold window, pitch-rate RMS was about `0.045 rad/s`

v3.4.38 therefore sets:

- `MC_PITCH_P=0.50`
- `MC_PITCHRATE_K=0.85`
- `MC_PITCHRATE_D` remains conservative at `0.0008`

The pitch-rate D value was not increased because the log does not show that D
was the root fix. The main correction is the attitude-loop pitch gain.

## Hover Thrust

qtail15 showed the 5 kg aircraft hovering near `0.20` motor output with total
thrust close to weight. v3.4.37's `MPC_THR_HOVER=0.30` was too high for the
measured plant and would bias altitude control upward if it loaded correctly.

v3.4.38 sets:

- `MPC_THR_HOVER=0.22`

## Fixed-Wing Phase

qtail15's fixed-wing phase should not be used to finalize FW gains because PX4
did not load the canted control-allocation axes or v3.4.37 FW/transition
defaults. Still, the symptoms are useful:

- transition reached FW state at about `268.6 s`
- repeated roll attitude failures started around `279 s`
- altitude climbed from roughly `35 m` to over `600 m` before RTL and then much
  higher during RTL
- TECS throttle demand was mostly near zero while motors still moved for
  differential-thrust attitude control

The next FW test must first prove the ULog loaded:

- `CA_ROTOR0_AX=-0.078`, `CA_ROTOR0_AY=0.040`, `CA_ROTOR0_AZ=-0.996`
- `CA_ROTOR1_AX=0.078`, `CA_ROTOR1_AY=-0.040`, `CA_ROTOR1_AZ=-0.996`
- `CA_ROTOR2_AX=-0.078`, `CA_ROTOR2_AY=-0.040`, `CA_ROTOR2_AZ=-0.996`
- `CA_ROTOR3_AX=0.078`, `CA_ROTOR3_AY=0.040`, `CA_ROTOR3_AZ=-0.996`
- `VT_ARSP_TRANS=24`
- `FW_AIRSPD_TRIM=28`
- `FW_THR_TRIM=0.16`
- `FW_THR_MAX=0.40`

If those values are not present, the run should be stopped and repeated after a
PX4 parameter reset.

## Throttle Display

The QGC or cockpit throttle display showing `0%` is not sufficient evidence that
the motors are inactive. TruthCapture showed:

- cockpit throttle datarefs stayed at `0`
- actual X-Plane `throttle_used_ratio` was active
- prop RPM and point thrust were active
- PX4 `actuator_motors` was active

For this plugin path, judge motor activity from ULog `actuator_motors` and
TruthCapture `sim/flightmodel2/engines/throttle_used_ratio`, prop RPM, and
`POINT_thrust`.

## Required Next Test Procedure

1. Pull the PX4 `px4xplane-sitl` branch after v3.4.38 is pushed.
2. Run `make distclean` once before the next QuadTailsitter test. Do not use
   only `make clean` after airframe parameter changes.
3. Build/run `make px4_sitl_default xplane_qtailsitter`.
4. Before judging flight behavior, verify the ULog initial parameters match the
   v3.4.38 sanity list in `QUADTAILSITTER_XPLANE12_TEST.md`.
5. Fly MC only first:
   - takeoff/hold
   - one short Go-To
   - one larger Go-To
   - one MC orbit
   - land
6. Only if MC is clean, repeat with a transition run.

## References

- PX4 Multicopter PID tuning guide:
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html
- PX4 airspeed configuration:
  https://docs.px4.io/main/en/config/airspeed.html
- PX4 actuator/control allocation:
  https://docs.px4.io/main/en/config/actuators.html
- PX4 tailsitter VTOL overview:
  https://docs.px4.io/main/en/frames_vtol/tailsitter.html
