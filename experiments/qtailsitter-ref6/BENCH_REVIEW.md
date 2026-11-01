# Optional BENCH Evidence Review

This is NOT required to boot diagnostics or capture an initial SITL hover.
`bash run.sh bench` truthfully reports pending measurements without this file.
After measuring in X-Plane, a reviewer may supply JSON with these fields:

```json
{
  "acf_sha256": "ACTUAL LOADED ACF HASH",
  "operator": "NAME", "evidence_reference": "BENCH LOG/CAPTURE PATH",
  "prop_geometry_expert_reviewed": false,
  "mass_cg_measured": false,
  "force_moment_signs_confirmed": false,
  "static_thrust_q_over_t_reviewed": false,
  "loaded_accel_decel_reviewed": false,
  "measured_max_rpm": [],
  "measured_max_shaft_power_w": [],
  "measured_hover_thrust_n": []
}
```

Populate from actual measurements, not analytical design estimates; mark reviews
true only after reviewing evidence. Each numeric array needs four positive finite
per-motor measurements. RPM and power are checked against design ceilings, and
hover thrust against weight. These checks do not establish sufficient control
margin, correct moment signs, calibrated dynamics, or flight readiness. Reviews
are explicit human statements, not independent validation. The report always
keeps `flight_validated: false`, `fw_release: false` and hover observation pending.

```bash
bash run.sh bench --bench-measurements "$PWD/bench-measurements.json"
```

An optional `--bench-measurements FILE` on `hover` records that review with the
flight inputs. No fabricated operator JSON is needed to launch the first staged
SITL capture. A completed BENCH review is not a transition release; stop, review
the hover capture and obtain separate explicit consent before any future FW test.
