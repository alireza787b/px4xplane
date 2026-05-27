# px4xplane Docs

This index is the stable entry point for user and developer documentation.

## User Guides

- [Build](BUILD.md)
- [Build System](BUILD_SYSTEM.md)
- [Alia X-Plane 12 Test Card](ALIA_XPLANE12_TEST.md)
- [Ehang 184 X-Plane 12 Test Card](EHANG184_XPLANE12_TEST.md)
- [QuadTailsitter X-Plane 12 Test Card](QUADTAILSITTER_XPLANE12_TEST.md)
- [Cessna 172 X-Plane 12 Test Card](CESSNA172_XPLANE12_TEST.md)
- [Custom Airframe Config](custom-airframe-config.md)
- [Config Editor](config-editor.html)

## Developer Notes

- [Developer Guide](DEVELOPER.md)
- [Config Schema](developer/config-schema.md)
- [Alia Demo Readiness Plan](plans/2026-05-07-alia-demo-readiness-plan.md)

## Reports

- [Report v12 - Alia Validation Follow-Up](reports/report_v12.md)
- [Report v13 - Alia Retest Regression Triage](reports/report_v13.md)
- [Report v14 - Offline Work Plan](reports/report_v14.md)
- [Report v15 - Config Safety Slice](reports/report_v15.md)
- [Report v16 - Runtime Config Validation UI Slice](reports/report_v16.md)
- [Report v17 - Config Schema Metadata Slice](reports/report_v17.md)
- [Report v18 - Static Config Editor Prototype](reports/report_v18.md)
- [Report v19 - Config Editor Menu Link Slice](reports/report_v19.md)
- [Report v20 - v3.4.8 Release Build Recovery](reports/report_v20.md)
- [Report v21 - Alia v3.4.9 Vertical Tune](reports/report_v21.md)
- [Report v22 - Alia v3.4.10 Regression Recovery](reports/report_v22.md)
- [Report v23 - Alia Test5 Crash Recovery](reports/report_v23.md)
- [Report v24 - Alia Test6 Fixed-Wing Tuning](reports/report_v24.md)
- [Report v25 - Alia Test7 Orbit Recovery](reports/report_v25.md)
- [Report v26 - Alia Final Polish and Ehang Prep](reports/report_v26.md)
- [Report v27 - Alia Bank Recovery and Ehang Nav Tuning](reports/report_v27.md)
- [Report v28 - Alia Transition Brake and Ehang Orbit Tuning](reports/report_v28.md)
- [Report v29 - Alia Airspeed and Ehang Orbit Evidence](reports/report_v29.md)
- [Report v30 - Alia Brake Recovery and Ehang Orbit Damping](reports/report_v30.md)
- [Report v31 - Alia Takeoff Gate and Ehang Roll Recovery](reports/report_v31.md)
- [Report v32 - SITL Connection Reliability](reports/report_v32.md)
- [Report v33 - v3.4.20 Regression and evtol6 Follow-Up](reports/report_v33.md)
- [Report v34 - evtol7 Milestone and QuadTailsitter Prep](reports/report_v34.md)
- [Report v35 - QuadTailsitter qtail1 Crash Recovery](reports/report_v35.md)
- [Report v36 - QuadTailsitter qtail2 Hover Dynamics Recovery](reports/report_v36.md)
- [Report v37 - QuadTailsitter qtail3 Yaw Authority Recovery](reports/report_v37.md)
- [Report v38 - QuadTailsitter qtail4 Yaw Allocation Recovery](reports/report_v38.md)
- [Report v39 - QuadTailsitter qtail5 Hover Authority Recovery](reports/report_v39.md)
- [Report v40 - QuadTailsitter qtail6 Hover Yaw Polish](reports/report_v40.md)
- [Report v41 - QuadTailsitter qtail7 Orbit Damping](reports/report_v41.md)
- [Report v42 - QuadTailsitter qtail8 Agile MC Recovery](reports/report_v42.md)
- [Report v43 - QuadTailsitter qtail9 Go-To Smoothness](reports/report_v43.md)
- [Report v44 - QuadTailsitter qtail9 Transition Airspeed Recovery](reports/report_v44.md)
- [Report v45 - QuadTailsitter qtail10 FW Energy Recovery](reports/report_v45.md)
- [Report v46 - QuadTailsitter qtail11 Body-Axis Pitot and Path Recovery](reports/report_v46.md)
- [Report v47 - QuadTailsitter qtail12 FW Orbit and Pitot Density](reports/report_v47.md)
- [Report v48 - QuadTailsitter qtail13 FW Energy Follow-Up](reports/report_v48.md)
- [Report v49 - QuadTailsitter qtail14 5kg Retarget](reports/report_v49.md)
- [Report v50 - QuadTailsitter qtail15 Parameter Sync and MC Pitch Fix](reports/report_v50.md)
- [Report v51 - QuadTailsitter Design Guardrails and Pause-Safe Timestamps](reports/report_v51.md)
- [Report v52 - Ground-Stationary IMU Guard](reports/report_v52.md)
- [Report v53 - Ground-Stationary Kinematics Guard](reports/report_v53.md)
- [Report v54 - Stationary-Ground Sensor Contract](reports/report_v54.md)
- [Report v55 - qtail19 Baro Liveness and Landing Contact](reports/report_v55.md)
- [Report v56 - qtail20 MC Overspeed Recovery and 5kg Design Audit](reports/report_v56.md)
- [Report v57 - qtail21 MC Acceptance and First-Transition Gate](reports/report_v57.md)
- [Report v58 - qtail22 FW Energy and Back-Transition Recovery](reports/report_v58.md)
- [Report v59 - qtail23 Mission Recovery Root Cause and v3.4.47 Fixes](reports/report_v59.md)
- [Report v60 - qtail24 Back-Transition Dive and v3.4.48 Fixes](reports/report_v60.md)
- [Report v61 - qtail25 Acceptance Polish and v3.4.49 Fixes](reports/report_v61.md)
- [Report v62 - qtail26 Closure Candidate and v3.4.50](reports/report_v62.md)
- [Report v63 - qtail27 Manual RTL and Config-Driven Camera Controls](reports/report_v63.md)
- [Report v64 - Config Editor and Camera Workflow Closure](reports/report_v64.md)
- [Report v65 - Config Editor UX Fixes and Cessna 172 Baseline](reports/report_v65.md)
- [Report v66 - Cessna 172 Runway Steering and Flare Recovery](reports/report_v66.md)

## Current Policy

- Use the `v3.4.54` package for the next Cessna 172 runway-mission validation.
  This package intentionally makes `Cessna172` the active config so testers do
  not have to switch away from Alia manually during this slice.
- Use the Cessna test card for auto runway takeoff, mission, and autoland.
  Confirm X-Plane parses Cessna channel 5 and the ULog has
  `PWM_MAIN_FUNC6=440` before judging runway steering or rollout behavior.
- QuadTailsitter closure remains based on qtail26/qtail27. qtail27 proved the
  v3.4.50 flight tune was loaded, but manual RTL started auto back-transition at
  the old `75 m` descent altitude and consumed that margin.
- Keep the qtail26/qtail27 bridge, ACF, back-transition timing, and 5 kg
  aircraft model unchanged unless new evidence points directly at the ACF
  physics.
- Use high-margin mission VTOL Land/back-transition altitudes; v3.4.51+ raises
  manual RTL to `RTL_RETURN_ALT=180` and `RTL_DESCEND_ALT=150`.
- The QuadTailsitter contact gear remains fixed for stability. v3.4.46 hides
  the large rendered legs; v3.4.52 keeps config-driven plugin camera views for
  all packaged airframes plus the previous quick-look presets and
  aircraft-selection icons. A richer operator cockpit/panel remains a later
  Plane Maker-authored visual slice.
- Force a PX4 parameter reset before judging any QuadTailsitter test; stale SITL
  `parameters.bson` values can override airframe `set-default` changes. qtail20
  did not load the intended v3.4.43 acceleration/jerk defaults.
- Run `tools/check_px4_airframe_params.py` on the returned ULog before tuning.
  If the canted `CA_ROTOR*_AX/AY/AZ`, `SYS_HAS_NUM_ASPD`, `MPC_THR_HOVER`,
  `MPC_XY_CRUISE`, `MPC_ACC_HOR`, `MPC_JERK_AUTO`, `VT_ARSP_TRANS`, or FW
  throttle values do not match the current airframe file, the flight is a setup
  mismatch, not valid tuning evidence.
- Alia and Ehang are now milestone airframes, but still require `distclean`
  after airframe-file changes before judging regressions.
- QuadTailsitter aircraft assets are now source-controlled under
  `aircraft/QuadTailsitter/`. Use that folder for packages instead of local
  `PB50` or ad hoc aircraft copies.
- Alia lift-prop braking is disabled by default. The generic brake policy still
  exists, but `feather`, `hard_lock`, and `prop_separate` are controlled A/B
  options only.
- Use `tools/validate_config.py config/config.ini` before packaging custom
  mappings.
- Use the X-Plane menu `Advanced > Validate Config` after changing the active
  mapping in-sim. Use `Advanced > Open Config Editor` for schema-backed editing;
  the plugin still reads only `64/config.ini`.
