"""Auditable compact tailsitter screening; predictions are not flight validation."""
import math

G = 9.80665
MASS = 2.0
SPAN = .6
CHORD = .1
SWEEP = math.radians(26.7)
AREA = 4 * (SPAN / 2 / math.cos(SWEEP)) * CHORD
NP_Y = -.25
CG_Y = NP_Y + .05 * CHORD
HUB_Y = -.20
MOTOR_HP = 500 / 745.699872


def components():
    parts = []
    for x, z in ((.3, -.15), (-.3, .15), (-.3, -.15), (.3, .15)):
        parts.append({'name': 'motor', 'mass_kg': .146, 'position_m': [x, HUB_Y, z],
                      'box_m': [.0371, .0565, .0371]})
    parts += [
        {'name': 'battery', 'mass_kg': .45, 'position_m': [0, 0, 0], 'box_m': [.05, .14, .05]},
        {'name': 'ESCs and wiring', 'mass_kg': .12, 'position_m': [0, NP_Y, 0], 'box_m': [.25, .05, .1]},
        {'name': 'avionics', 'mass_kg': .06, 'position_m': [0, NP_Y, 0], 'box_m': [.06, .03, .04]},
        {'name': 'airframe', 'mass_kg': .5, 'position_m': [0, NP_Y, 0], 'box_m': [.6, .10, .30]},
        {'name': 'equipment allowance', 'mass_kg': .286, 'position_m': [0, NP_Y, 0], 'box_m': [.06, .06, .06]},
    ]
    # Battery placement solves the desired CG from component masses; no hidden
    # ballast is added. Verify the result lies inside the proposed body.
    fixed_y = sum(p['mass_kg'] * p['position_m'][1] for p in parts if p['name'] != 'battery')
    battery = next(p for p in parts if p['name'] == 'battery')
    battery['position_m'][1] = (MASS * CG_Y - fixed_y) / battery['mass_kg']
    assert -.4 < battery['position_m'][1] < -.1
    return parts


def inertia(parts, cg):
    tensor = [[0.0]*3 for _ in range(3)]
    for p in parts:
        mass = p['mass_kg']
        d = [p['position_m'][i]-cg[i] for i in range(3)]
        box = p['box_m']
        for i in range(3):
            tensor[i][i] += mass * sum(box[j]**2 for j in range(3) if j != i)/12
            for j in range(3):
                tensor[i][j] += mass * ((sum(v*v for v in d) if i == j else 0) - d[i]*d[j])
    return tensor


def propulsion(voltage, density):
    # Power-led NACA2412 two-blade surrogate from the reproducible BEM script.
    # Motor torque constant from KV; no-load current and resistance are BSC
    # manufacturer specifications. This is not an identified ESC speed loop.
    kt = 60/(2*math.pi*1000)
    def point(rpm):
        thrust = 23.954*(rpm/12000)**2*density/1.225
        power = 573.45*(rpm/12000)**3*density/1.225
        torque = power/(rpm*math.pi/30)
        current = torque/kt + .7
        return thrust, power, current, rpm/1000 + current*.0295
    lo, hi = 100., 13000.
    for _ in range(50):
        mid = (lo+hi)/2
        _, p, _, v = point(mid)
        if p <= 500 and v <= voltage:
            lo = mid
        else:
            hi = mid
    thrust, power, current, required = point(lo)
    return {'rpm': lo, 'thrust_n': thrust, 'shaft_w': power,
            'estimated_motor_current_a': current, 'required_motor_voltage_v': required,
            'voltage_v': voltage, 'density_kg_m3': density,
            'warning': 'DC equivalent plus BEM; installation, ESC losses, Reynolds variation and thermal duration are not validated'}


def screen():
    parts = components()
    mass = sum(p['mass_kg'] for p in parts)
    cg = [sum(p['mass_kg']*p['position_m'][i] for p in parts)/mass for i in range(3)]
    envelope = []
    for m in (1.75, 2., 2.25, 2.5):
        for margin in (0., .05, .1):
            for rho in (1.225, 1.059, 1.):
                for volts in (25.2, 22.2, 19.8):
                    p = propulsion(volts, rho)
                    # Equal thrust plus +/- fore/aft differential to trim the
                    # longitudinal CG offset. Assumes lift acts at screened NP.
                    moment = m*G*margin*CHORD
                    delta_each = moment/(4*.15)
                    hover_each = m*G/4
                    reserve = min(hover_each-delta_each, p['thrust_n']*.8-hover_each-delta_each)
                    envelope.append({'mass_kg': m, 'static_margin_candidate': margin,
                        'rho': rho, 'voltage': volts,
                        'stall_estimate_mps_clmax1': math.sqrt(2*m*G/(rho*AREA)),
                        'stall_estimate_mps_clmax1p2': math.sqrt(2*m*G/(rho*AREA*1.2)),
                        'tw_derated20pct': .8*4*p['thrust_n']/(m*G),
                        'trim_moment_nm_estimate': moment, 'differential_reserve_each_n': reserve,
                        'screen_pass': .8*4*p['thrust_n']/(m*G) >= 2.5 and reserve > 0})
    return {'mass_kg': mass, 'components': parts, 'cg_xplane_m': cg,
        'inertia_tensor_xplane_kg_m2': inertia(parts, cg),
        'main_area_m2': AREA, 'span_m': SPAN, 'chord_m': CHORD,
        'neutral_point_estimate_y_m': NP_Y,
        'static_margin_seed': .05,
        'neutral_point_limit': 'Wing-only quarter-chord estimate after disabling secondary surfaces; body and propwash can shift it. Cm-alpha and trim require X-Plane measurement.',
        'propulsion_grid': [propulsion(v,r) for v in (25.2,22.2,19.8) for r in (1.225,1.059,1.)],
        'envelope': envelope, 'flight_validated': False,
        'assumptions': ['6S 3000mAh battery budget 450g, not a selected battery product',
            '500W shaft/motor conservative design cap, not a proven continuous manufacturer rating',
            '10x8E-inspired surrogate, not manufacturer 10x5x3 test prop',
            '20% thrust derating is a screening allowance, not a statistical bound',
            'No payload is added solely to damp a control problem',
            'No supplied component mass/position is an actual hardware measurement']}
