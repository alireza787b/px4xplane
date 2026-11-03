# QuadTailsitter X-Plane 12 Test Card

Use the public legacy QuadTailsitter reference with its matching model.
Model/profile development details are in the
[compact developer guide](compact-qtailsitter.md), not the normal setup flow.

1. Follow the [Quick Start](../README.md#quick-start) using official PX4 `main`.
   Select the matching X-Plane aircraft and plugin section `[QuadTailsitter]`.
2. Select `xplane_qtailsitter` and verify `SYS_AUTOSTART=5021`. Confirm the
   actual PX4 defaults and saved parameters; the target ID alone is not enough.
   The guided launcher keeps this airframe's state separate and preserves tuning.
3. Confirm QGC shows a connected vehicle and wait for estimator readiness.
   The X-Plane TCP connection alone does not confirm QGC telemetry.
4. Begin with hover, a short movement/stop and Land on a changed installation.
   Never use these SITL values on hardware.
5. Allow ample terrain/altitude margin for transition checks. Settle in
   fixed-wing flight and check one back-transition before evaluating RTL.
6. Judge body rates, quaternions and actual height, not near-vertical Euler
   roll display alone. End the simulation for growing departure or non-decay.
7. Verify controlled descent, touchdown and automatic disarm.

Record the source revision, plugin version, model/profile identity, PX4 output
and relevant ULog results. This checklist does not claim a new completed flight
or full-envelope acceptance. See [Troubleshooting](index.md#troubleshooting)
for connection or saved-parameter issues.
