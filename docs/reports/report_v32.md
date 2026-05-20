# Report v32 - SITL Connection Reliability

Scope: review user feedback that PX4 SITL sometimes connects immediately and
sometimes waits several seconds, then harden the connection lifecycle without
changing the accepted Alia/Ehang flight-tuning baseline.

## Finding

px4xplane uses the correct PX4 simulator TCP shape: the plugin opens a server
socket on TCP `4560`, and PX4 `simulator_mavlink` connects as the client. The
plugin already polls `accept()` non-blocking every X-Plane flight-loop frame, so
X-Plane should not freeze while waiting.

A `5-10 s` wait can still be normal when PX4 is started before the plugin begins
listening. In that order, PX4 may be between TCP retry attempts when the plugin
opens port `4560`; the next PX4 retry then connects suddenly. The plugin cannot
force PX4's client retry cadence, but it can make the state obvious and keep the
socket lifecycle clean.

## Fixes

- While waiting, the menu now says `Cancel SITL Connection Wait`. Selecting it
  closes the listening socket. Previously the menu still looked like a second
  connect action.
- Plugin unload/disable now closes a waiting listener, not only an established
  connection.
- Accepted PX4 sockets are explicitly non-blocking and use `TCP_NODELAY`.
- Send, receive, and select socket failures now disconnect cleanly and update
  the data-window last event.
- The waiting timeout now uses flight-loop wall time instead of simulator time,
  so the `30 s` timeout matches what the user experiences even if the sim is
  paused or slow.
- Removed `SO_REUSEPORT`. Quick reconnect still uses `SO_REUSEADDR`, but the
  plugin should not silently allow multiple listeners to share TCP `4560`.

## Expected User Behavior

Both start orders are supported:

- Start X-Plane/plugin first, click connect, then start PX4. The HUD waits and
  should accept PX4 as soon as it connects.
- Start PX4 first, then click connect in X-Plane. PX4 may connect immediately or
  after its next retry attempt. A few seconds is acceptable; more than `30 s`
  is treated as a timeout and the user can retry.

## Flight-Tuning Impact

None. This slice does not change Alia or Ehang PX4 parameters, `config.ini`
airframe mappings, prop-brake defaults, sensor signs, MAVLink units, TECS,
NPFG, multicopter gains, or transition settings.

The next flight test should still verify the v3.4.19 flight-tuning points that
are carried unchanged into v3.4.20:

- Alia PX4 log shows `SYS_AUTOSTART=5020` and `MPC_XY_ERR_MAX=10`.
- Ehang PX4 log shows `SYS_AUTOSTART=5010`, `MC_ROLL_P=0.3`,
  `MC_PITCH_P=0.3`, and `MPC_XY_VEL_D_ACC=1.5`.
- X-Plane log shows `px4xplane: Version: v3.4.20`.
- During the SITL wait, HUD/menu/data-window state is clear and cancellable.
