# px4xplane Docs

This index is the stable entry point for user and developer documentation.

## User Guides

- [Build](BUILD.md)
- [Build System](BUILD_SYSTEM.md)
- [Alia X-Plane 12 Test Card](ALIA_XPLANE12_TEST.md)
- [Custom Airframe Config](custom-airframe-config.md)
- [Config Editor](config-editor.html)

## Developer Notes

- [Developer Guide](DEVELOPER.md)
- [Config Schema](developer/config-schema.md)
- [Alia Demo Readiness Plan](plans/2026-05-07-alia-demo-readiness-plan.md)

## Reports

- [Report v14 - Offline Work Plan](reports/report_v14.md)
- [Report v15 - Config Safety Slice](reports/report_v15.md)
- [Report v16 - Runtime Config Validation UI Slice](reports/report_v16.md)
- [Report v17 - Config Schema Metadata Slice](reports/report_v17.md)
- [Report v18 - Static Config Editor Prototype](reports/report_v18.md)
- [Report v19 - Config Editor Menu Link Slice](reports/report_v19.md)
- [Report v20 - v3.4.8 Release Build Recovery](reports/report_v20.md)
- [Report v21 - Alia v3.4.9 Vertical Tune](reports/report_v21.md)
- [Report v22 - Alia v3.4.10 Regression Recovery](reports/report_v22.md)

## Current Policy

- Use the `v3.4.10` Alia recovery package for the next retest, and reset PX4
  SITL parameters with `distclean` before judging the package.
- Use `tools/validate_config.py config/config.ini` before packaging custom
  mappings.
- Use the X-Plane menu `Advanced > Validate Config` after changing the active
  mapping in-sim.
