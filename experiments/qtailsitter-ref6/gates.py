"""Record measured BENCH release gates. Analytical design numbers are not measurements."""
import json
import math
from pathlib import Path

from ref6 import HERE, NAME, digest


def evaluate(design, measurements):
    data = measurements or {}
    gates = {}
    for key in ('prop_geometry_expert_reviewed', 'mass_cg_measured', 'force_moment_signs_confirmed',
                'static_thrust_q_over_t_reviewed', 'loaded_accel_decel_reviewed'):
        gates[key] = data.get(key) is True
    def four(key):
        values = data.get(key, [])
        return values if isinstance(values, list) and len(values) == 4 and all(
            type(v) in (int, float) and math.isfinite(v) and v > 0 for v in values) else None
    rpm = four('measured_max_rpm')
    power = four('measured_max_shaft_power_w')
    thrust = four('measured_hover_thrust_n')
    gates['rpm_ceiling'] = bool(rpm and max(rpm) <= design['rpm_limit'])
    gates['shaft_power_ceiling'] = bool(power and max(power) <= design['shaft_power_budget_w']/4)
    gates['hover_thrust_supports_weight'] = bool(thrust and sum(thrust) >= design['mass_kg']*9.80665)
    gates['operator_and_evidence'] = bool(data.get('operator') and data.get('evidence_reference'))
    gates['acf_hash'] = data.get('acf_sha256') == design['acf_sha256']
    return {'aircraft': NAME, 'acf_sha256': design['acf_sha256'], 'gates': gates,
            'bench_review_complete': all(gates.values()), 'flight_validated': False,
            'fw_release': False, 'hover_result': 'PENDING observed controlled hover',
            'note': 'Pending measurements may be collected in a consented SITL hover experiment. This report is not a launch/readiness gate or independent calibration proof. No EKF overrides.'}


def report(measurements=None):
    design = json.loads((HERE/'design.json').read_text())
    data = json.loads(Path(measurements).read_text()) if measurements else None
    result = evaluate(design, data)
    if measurements:
        result.update(measurements_path=str(Path(measurements).resolve()), measurements_sha256=digest(Path(measurements)))
    output = HERE/'bench-report.json'
    output.write_text(json.dumps(result, indent=2)+'\n')
    if measurements:
        print('BENCH report:', output)
        for key, passed in result['gates'].items():
            print('  '+key+': '+('PASS (operator evidence)' if passed else 'NOT VERIFIED'))
    else:
        print('Bench measurements not yet verified; details saved to bench-report.json (not a measured failure).')
    return result
