# px4xplane Docs

This index is the stable entry point for user and developer documentation.

## User Guides

- [Build](BUILD.md)
- [Build System](BUILD_SYSTEM.md)
- [Alia X-Plane 12 Test Card](ALIA_XPLANE12_TEST.md)
- [Ehang 184 X-Plane 12 Test Card](EHANG184_XPLANE12_TEST.md)
- [QuadTailsitter X-Plane 12 Test Card](QUADTAILSITTER_XPLANE12_TEST.md)
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

## Current Policy

- Use the `v3.4.25` package for the next QuadTailsitter hover-only recovery
  test. Alia and Ehang are now milestone airframes, but still require
  `distclean` after airframe-file changes before judging regressions.
- QuadTailsitter aircraft assets are now source-controlled under
  `aircraft/QuadTailsitter/`. Use that folder for packages instead of local
  `PB50` or ad hoc aircraft copies.
- Alia lift-prop braking is disabled by default. The generic brake policy still
  exists, but `feather`, `hard_lock`, and `prop_separate` are controlled A/B
  options only.
- Use `tools/validate_config.py config/config.ini` before packaging custom
  mappings.
- Use the X-Plane menu `Advanced > Validate Config` after changing the active
  mapping in-sim.
