# Ref6 Operator Gates

Simulation experiment only. Do not use on hardware. No stability guarantee.

1. Wait for `REF6 PARAMETER GATE PASSED`. If absent or any parameter gate fails,
   do not arm. The startup script requests shutdown on comparison failure.
2. Run `mavlink status`, `listener vehicle_status -n 1`, `param show SYS_AUTOSTART`
   and `param show`. Verify 5021 and the recorded `profile.sh` values. Verify QGC
   heartbeat and `gcs_connection_lost: False`; a running MAVLink endpoint alone
   is insufficient. Verify Ref6 selected and recorded in the truth manifest.
3. Verify upright initial attitude, correct motor responses, position/altitude
   estimate, and no arming warnings. Abort on any discrepancy; never force arm.
4. After structural/identity checks and explicit SITL experiment consent, type
   `commander takeoff`. Wait for takeoff completion and observe 20 seconds of hover.
   Confirm takeoff altitude 180 m for `flight` or 50 m for `hover`. Initial checkpoint:
   no growing oscillation, roughly less than 5 degrees roll/pitch variation and
   altitude within 5 m after settling. Abort on tilt growth, estimator warnings,
   saturation or loss of control. Do not transition unless hover is controlled.
5. In `hover` mode, skip to landing. In `flight` mode only, at 180 m clearance,
   type `commander transition` ONCE. Wait for confirmed FW state; observe at least
   20 seconds. Do not repeatedly toggle because a command seems delayed.
   QGC Hold uses the inherited 1300 m orbit, not an optimized Ref6 radius. For path
   evidence use two well-separated, same-altitude waypoints, not abrupt close turns.
   Do not change speed or gains. While on a settled shallow bank or wings-level,
   with at least 180 m clearance, type `commander transition` ONCE for MC, then
   `listener vtol_vehicle_status -n 1`. Wait for upright recovery; observe 20 seconds.
   A timer-ended mode change is not proof of recovery. These are unvalidated SITL
   experiments, not release approvals. Preserve the first failure rather than
   attempting a prolonged uncontrolled recovery.
6. Abort on increasing attitude error, yaw sign error, motor saturation, uncontrolled
   descent or estimator warnings. Do not override EKF or repeat prolonged recovery.
7. Once upright and controlled: `commander land`, then
   `listener vehicle_status -n 1`. Verify land state and sustained descent; no RTL.
   Run `listener trajectory_setpoint -n 1` as well. If descent fails to begin within
   15 seconds after settling, preserve that interval and stop the experiment; do
   not repeatedly change modes/gains. Otherwise let controlled landing complete
   and confirm disarmed, then `shutdown`.
8. Stop TruthCapture AFTER PX4 exits and BEFORE confirming collection. Preserve
   all failures as evidence. For an uncontrolled simulation, stop the experiment
   using simulator pause/PX4 shutdown as appropriate; do not interpret a stopped
   process as a successful landing.
