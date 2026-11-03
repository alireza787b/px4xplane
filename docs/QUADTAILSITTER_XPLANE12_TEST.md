# QuadTailsitter Model-Pairing Check

There are two incompatible model/profile generations. Read the
[compact migration guide](compact-qtailsitter.md) before selecting one.
The draft compact defaults are not a generic retune for the old 5 kg aircraft.

1. Check the model filename, ACF hash, selected plugin section and PX4 defaults.
   The compact QuadTailsitter model is supplied privately by its owner.
2. Use separate saved parameters. Verify the startup parameter gate, source
   revision and `SYS_AUTOSTART=5021`; do not infer the tune from the ID alone.
3. Connect QGC and wait for estimator readiness without raising bias limits.
4. Begin with hover, a short movement/stop and Land before trying transitions
   on a changed build or installation. Never use these SITL values on hardware.
5. For a further flight check, allow substantial terrain/altitude margin,
   settle in FW and command one back-transition before evaluating RTL geometry.
   Existing 1300 m loiter defaults do not override a mission's explicit radius.
6. Judge body rates, quaternions and actual height, not the near-vertical Euler
   roll display alone. End the simulation for growing departure or non-decay.
7. Verify continuous commanded descent, touchdown and automatic disarm.

The R16 owner test completed mission/FW/RTL/Land, but residual oscillation and
back-transition climb remain. This card is not a production acceptance claim.
Additional full-envelope or other-airframe tuning is separate work.

If investigating a regression, retain the ULog, source/profile/model identities,
X-Plane Log.txt and optional independent truth. Do not upload aircraft-bearing
archives publicly; arrange private reviewer access or publish sanitized metrics.
