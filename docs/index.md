# px4xplane Docs

This index is the stable entry point for user and developer documentation.

## User Guides

- [Build](BUILD.md)
- [Build System](BUILD_SYSTEM.md)
- [Alia X-Plane 12 Test Card](ALIA_XPLANE12_TEST.md)
- [Custom Airframe Config](custom-airframe-config.md)

## Developer Notes

- [Developer Guide](DEVELOPER.md)
- [Alia Demo Readiness Plan](plans/2026-05-07-alia-demo-readiness-plan.md)

## Reports

- [Report v14 - Offline Work Plan](reports/report_v14.md)
- [Report v15 - Config Safety Slice](reports/report_v15.md)
- [Report v16 - Runtime Config Validation UI Slice](reports/report_v16.md)

## Current Policy

- Keep Alia parameters frozen until the next complete v3.4.6 baseline retest.
- Use `tools/validate_config.py config/config.ini` before packaging custom
  mappings.
- Use the X-Plane menu `Advanced > Validate Config` after changing the active
  mapping in-sim.
