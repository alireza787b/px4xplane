#!/usr/bin/env python3
"""Build a separate, unvalidated SITL reference. Never edits the source aircraft."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil

from analyze_qtailsitter_design import FT_TO_M, LB_TO_KG, G, parse_acf, as_float, estimate_wing_area_m2
import qtailsitter_sizing as sizing

NAME = "QuadTailsitterRef6"
MASS_KG = sizing.MASS
MAIN_AREA_M2 = sizing.AREA
DIAMETER_M = 0.254
PITCH_M = 0.2032
RPM_LIMIT = 13000
MOTOR_KW = 0.5
EFFICIENCY = 0.85
AUX_W = 50
# Manufacturer analytical data, not thrust-stand measurements or an X-Plane fit.
APC_STATIC = ((10000, 18.148, 334.622, 0.320),
              (12000, 26.271, 577.465, 0.460))
APC_URL = "https://www.apcprop.com/files/PER3_10x8E.dat"
# Independent blade-element/momentum screening using the supplied first
# NACA2412 polar, 160 annuli and tip loss. Reproduce with propulsion_bem.py
# in the test bundle. These are predictions, NOT simulator measurements.
BEM_STATIC_RPM = 12000
BEM_STATIC_THRUST_N = 23.954
BEM_STATIC_POWER_W = 573.45


def compact_geometry(values, result):
    chord_scale = sizing.CHORD / (as_float(values, '_wing/0/_Croot') * FT_TO_M)
    root_y = sizing.NP_Y / FT_TO_M
    for i in range(4):
        p = f'_wing/{i}'
        old_x = as_float(values, p+'/_part_x')
        old_y = as_float(values, p+'/_part_y')
        old_z = as_float(values, p+'/_part_z')
        xs = [abs(float(v)-old_x) for k,v in values.items() if k.startswith(p+'/_geo_xyz/') and k.endswith(',0')]
        scale = (sizing.SPAN / 2 / FT_TO_M) / max(xs)
        for field in ('_Croot', '_Ctip'):
            result[p+'/'+field] = number(sizing.CHORD / FT_TO_M)
        for j in range(11):
            result[f'{p}/_chord_for_RIB/{j}'] = number(sizing.CHORD / FT_TO_M)
        result[p+'/_semilen_SEG'] = number(as_float(values,p+'/_semilen_SEG')*scale)
        result[p+'/_part_y'] = number(root_y)
        result[p+'/_part_z'] = number(old_z*scale)
        for key,value in values.items():
            match = re.fullmatch(re.escape(p)+r'/_geo_xyz/(\d+),(\d+),([012])', key)
            if not match:
                continue
            station, section, axis = map(int, match.groups())
            if station >= int(as_float(values,p+'/_s_dim')) or section >= int(as_float(values,p+'/_r_dim')):
                continue
            if axis == 0:
                result[key] = number(float(value)*scale)
            elif axis == 1:
                # Unused mesh entries remain zero, not displaced into geometry.
                result[key] = number(root_y + (float(value)-old_y)*chord_scale)
            else:
                xkey=key[:-1]+'0'
                x=as_float(values,xkey)
                center=old_z+abs(x-old_x)*math.tan(math.radians(as_float(values,p+'/_sweep_design')))
                result[key]=number(center*scale+(float(value)-center)*chord_scale)
        # 500 W is converted to Plane Maker's horsepower field. It is NOT 0.5
        # in this serialized field; the previous builder's kW assumption was wrong.
        result[f'_engn/{i}/_power_max_limit'] = number(sizing.MOTOR_HP)
    # Remove inherited secondary aerodynamic panels rather than silently carrying
    # their old trim contribution into a smaller tailless reference.
    for i in (8,9,10,11):
        p=f'_wing/{i}'
        for key in values:
            if (key.startswith(p+'/_geo_xyz/') or key.startswith(p+'/_chord_for_RIB/')) and not key.endswith(('_count', '/count')):
                result[key]=number(0)
        for field in ('_semilen_SEG','_Croot','_Ctip'):
            result[p+'/'+field]=number(0)
    hubs=((.3,-.15),(-.3,.15),(-.3,-.15),(.3,.15))
    for i,(x,z) in enumerate(hubs):
        for group in (f'_engn/{i}',f'_blad/{i}',f'_body/{16+i}'):
            for axis,value in (('x',x),('y',sizing.HUB_Y),('z',z)):
                result[group+'/_part_'+axis]=number(value/FT_TO_M)
    # Compact the inherited fuselage and non-motor fairing. Propeller meshes
    # and motor nacelles keep their physical dimensions.
    for i in (0,21):
        p=f'_body/{i}'
        for key,value in values.items():
            if (key.startswith(p+'/_geo_xyz/') and not key.endswith('_count')) or key in (p+'/_part_x',p+'/_part_y',p+'/_part_z',p+'/_part_rad'):
                result[key]=number(float(value)*.6)
    for i in range(4):
        for field in ('_gear_x','_gear_y','_gear_z','_leg_len'):
            key=f'_gear/{i}/{field}'
            result[key]=number(as_float(values,key)*.6)
    result['acf/_cgY']=number(sizing.CG_Y/FT_TO_M)
    result['acf/_cgZ']=number(0)
    result['acf/_fixed_ref/0,1']=number(sizing.CG_Y/FT_TO_M)
    # X-Plane populated half of this payload capacity on the previous run:
    # 2 kg empty + 0.25 kg fixed. Keep the first comparison at the design mass.
    result['acf/_fixed_max/0']=number(0)
    result['acf/_m_max']=number(2.5/LB_TO_KG)
    result['acf/_battery_watt_hr_max']=number(22.2*3)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rewrite(text: str, replacements: dict[str, str]) -> str:
    seen = set()
    result = []
    for line in text.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[0] == "P" and parts[1] in replacements:
            key = parts[1]
            if key in seen:
                raise ValueError(f"Duplicate ACF field: {key}")
            seen.add(key)
            line = f"P {key} {replacements[key]}"
        result.append(line)
    missing = replacements.keys() - seen
    if missing:
        raise ValueError(f"Unsupported ACF; missing fields: {sorted(missing)}")
    return "\n".join(result) + "\n"


def number(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("Non-finite ACF value")
    return f"{value:.9f}"


def replacements_for(values: dict[str, str], blade_chord_scale: float) -> dict[str, str]:
    area = estimate_wing_area_m2(values)["main_panel_area_m2"]
    if area <= 0 or not 1 <= blade_chord_scale <= 5:
        raise ValueError("Invalid source wing area or blade chord scale")
    result = {
        "acf/_m_empty": number(MASS_KG / LB_TO_KG),
        "acf/_engine_rev_time_sec": number(0.1),
        "acf/_rotor_mi_rat": number(1400 / 2700),
        "acf/_fadec_EQ_rpm_limit": "1",
        "acf/_use_manual_prop_inc": "1",
    }
    for index in range(4):
        engine = f"_engn/{index}"
        result[f"{engine}/_RSC_redline_ENGN"] = number(RPM_LIMIT)
        result[f"{engine}/_RSC_max_pwr_ENGN"] = number(RPM_LIMIT)
        # Preserve the source aircraft's engine orientations verbatim. Do not
        # remove cant while changing propulsion performance.
        prop = f"_prop/{index}"
        result[f"{prop}/_num_blades"] = number(2)
        result[f"{prop}/_des_rpm_prp"] = number(12000)
        result[f"{prop}/_rad_mod"] = number(1)
        blade = f"_blad/{index}"
        old_radius = as_float(values, f"{blade}/_semilen_SEG")
        radius_ft = DIAMETER_M / 2 / FT_TO_M
        radial_scale = radius_ft / old_radius
        result[f"{blade}/_semilen_SEG"] = number(radius_ft)
        for key in ("_Croot", "_Ctip"):
            result[f"{blade}/{key}"] = number(as_float(values, f"{blade}/{key}") * blade_chord_scale)
        for rib in range(int(as_float(values, f"{blade}/_chord_for_RIB/count"))):
            key = f"{blade}/_chord_for_RIB/{rib}"
            result[key] = number(as_float(values, key) * blade_chord_scale)
        for station in range(10):
            # Ten section incidences correspond to eleven rib chords. The
            # baseline mesh's interior rib twist averages adjacent incidences.
            # Meaningful blade starts outside the declared r/R=0.2 hub.
            radius = DIAMETER_M / 2 * max((station + .5) / 10, 0.2)
            twist = math.degrees(math.atan2(PITCH_M, 2 * math.pi * radius))
            result[f"{blade}/_incidence/{station}"] = number(twist)
        # Propeller mesh is blade-local. Preserve camber while scaling chord,
        # radial extent and rotating each section to the new geometric pitch.
        def chord_angle(station):
            prefix = f"{blade}/_geo_xyz/{station},"
            y = as_float(values, prefix + "5,1") - as_float(values, prefix + "0,1")
            z = as_float(values, prefix + "5,2") - as_float(values, prefix + "0,2")
            return math.atan2(z, -y)
        root_angle = chord_angle(0)
        root_twist = math.radians(as_float(values, f"{blade}/_incidence/0"))
        for station in range(int(as_float(values, f"{blade}/_s_dim"))):
            sample = f"{blade}/_geo_xyz/{station},0,0"
            x = as_float(values, sample)
            fraction = max(0.2, min(1.0, x / old_radius))
            twist = math.degrees(math.atan2(PITCH_M, 2 * math.pi * DIAMETER_M / 2 * fraction))
            # Recover the actual old section orientation rather than guessing
            # interpolation between stored incidences. atan2(z, -y) decreases
            # under a positive YZ rotation, hence old angle minus target angle.
            angle = chord_angle(station) - root_angle - root_twist + math.radians(twist)
            for section in range(int(as_float(values, f"{blade}/_r_dim"))):
                prefix = f"{blade}/_geo_xyz/{station},{section}"
                y = as_float(values, prefix + ",1") * blade_chord_scale
                z = as_float(values, prefix + ",2") * blade_chord_scale
                result[prefix + ",0"] = number(as_float(values, prefix + ",0") * radial_scale)
                result[prefix + ",1"] = number(y * math.cos(angle) - z * math.sin(angle))
                result[prefix + ",2"] = number(y * math.sin(angle) + z * math.cos(angle))
    compact_geometry(values, result)
    return result


def parameter_profile(values: dict[str, str]) -> tuple[dict[str, float], dict[str, str]]:
    params = {}
    reasons = {}
    def add(name: str, value: float, reason: str):
        params[name] = value
        reasons[name] = reason

    # Actual 2 kg R6 hover/GoTo/Land HTE ranged around 0.26, without ballast.
    add("MPC_THR_HOVER", .26, "R6 measured HTE: hover median 0.2657, GoTo 0.2613, Land 0.2603 at actual 2kg. Rounded starting seed; estimator remains enabled.")
    for index in range(4):
        engine = f"_engn/{index}"
        add(f"CA_ROTOR{index}_PX", -as_float(values, f"{engine}/_part_z") * FT_TO_M, "Actual ACF hub position in MC FRD, meters.")
        add(f"CA_ROTOR{index}_PY", as_float(values, f"{engine}/_part_x") * FT_TO_M, "Actual ACF hub position in MC FRD, meters.")
        add(f"CA_ROTOR{index}_PZ", -(as_float(values, f"{engine}/_part_y") - as_float(values, "acf/_cgY")) * FT_TO_M, "Actual hub height relative to CG.")
        # Retain the pinned official airframe's direction vectors. These are
        # its existing cant model, not a newly inferred X-Plane Euler transform.
        directions = ((-.078,.040,-.996),(.078,-.040,-.996),
                      (-.078,-.040,-.996),(.078,.040,-.996))
        for axis, value in zip(('AX','AY','AZ'), directions[index]):
            add(f"CA_ROTOR{index}_{axis}", value, "Unchanged direction vector from pinned official 5021_xplane_qtailsitter; source engine angles and motor order retained. Physical sign validation remains required.")
        direction = as_float(values, f"_prop/{index}/_prop_dir")
        add(f"CA_ROTOR{index}_CT", 6.5, "PX4 default common allocation scale. CT=2 with this KM fell below the allocator's 0.05 weak-axis cutoff. Common scaling is removed by normalized allocation; not a measured maximum thrust.")
        km = BEM_STATIC_POWER_W / (BEM_STATIC_RPM * math.pi / 30) / BEM_STATIC_THRUST_N
        add(f"CA_ROTOR{index}_KM", direction * km, "BEM static Q/T prediction; rotation sign and actual Q/T must pass bench test.")
    # Hold unrelated gains fixed: revising a hover estimate must not silently
    # retune every axis. The previous rigid-body/cant-free model was insufficient.
    km = BEM_STATIC_POWER_W/(BEM_STATIC_RPM*math.pi/30)/BEM_STATIC_THRUST_N
    yaw_levers=[]
    for i in range(4):
        ax,ay,az=(params[f'CA_ROTOR{i}_A{a}'] for a in ('X','Y','Z'))
        norm=math.sqrt(ax*ax+ay*ay+az*az)
        px,py=(params[f'CA_ROTOR{i}_P{a}'] for a in ('X','Y'))
        yaw_levers.append(abs((px*ay-py*ax-params[f'CA_ROTOR{i}_KM']*az)/norm))
    yaw_scale=km/(sum(yaw_levers)/4)
    gains={'ROLL':(.060,.007954,.000636),
           'YAW':tuple(v*yaw_scale for v in (.105262,.02,.002105))}
    for axis,(p,integral,d) in gains.items():
        reason=('R8 post-back MC roll rate error ~0.21rad/s RMS with ~0.48s closed-loop correlation lag, motor headroom and small integral. P-only trial 0.031815->0.060; four local delayed-model replays suggest ~40% less rate error. I,D,K unchanged. In-sample screens are not a closed-position-loop stability guarantee; validate lateral MC motion before FW.' if axis=='ROLL' else
                'R5 yaw gains reduced by reaction-only/restored-cant yaw lever ratio; the new cant adds modeled authority. Trial compensation, not measured physical validation.')
        for suffix,value in (('P',p),('I',integral),('D',d),('K',1),('FF',0)):
            add(f'MC_{axis}RATE_{suffix}',round(value,6),reason)
    for suffix,value in (('P',.12),('I',.015),('D',.0012),('K',1)):
        add('MC_PITCHRATE_'+suffix,value,
            'R6 GoTo pitch-rate lag ~1.38s and error ~0.15rad/s RMS with motor headroom. Trial P-only step 0.06->0.12; I,D,K unchanged. Closed-loop evidence is not an identified optimum or a phase-margin guarantee.')
    add('MC_PITCHRATE_FF',0,'No identified pitch rate feedforward.')
    # Same physical rotors and allocator in both modes. Follow the pinned frame
    # transforms (FW roll <- MC yaw, pitch <- pitch, FW yaw <- MC roll) and cancel
    # downstream differential scaling. This equates unsaturated normalized PID
    # effort, NOT aerodynamic response, actuator bandwidth or stability margins.
    # Freeze the failed R8 FW trial during the isolated R9 MC-roll experiment.
    # In particular, do not silently copy the new MC roll P into FW yaw.
    fw_baseline={'RR':(.075360,.014318,.001507),
                 'PR':(.133333,.016667,.001333),
                 'YR':(.099422,.024856,.001987)}
    for fw,coefficients in fw_baseline.items():
        for term,value in zip(('P','I','D'),coefficients):
            add(f'FW_{fw}_{term}',round(value,6),
                'Frozen R8 FW trial coefficient, NOT a validated tune. Do not propagate the isolated R9 MC-roll change. FW/back-transition are on hold pending lateral MC validation.')
        add(f'FW_{fw}_FF',0,'No identified FW aerodynamic rate feedforward; preserve zero during first short characterization.')
    for axis,scale in (('R',.55),('P',.90),('Y',.32)):
        add(f'VT_FW_DIFTHR_S_{axis}',scale,'Unchanged pinned 5021 differential torque scaling, explicit for rate-seed reproducibility.')
    add('FW_ARSP_SCALE_EN',0,'Rotor-only torque, no movable aerodynamic surfaces: disable aerodynamic surface V^-2 rate-gain schedule per PX4 parameter guidance. Prop authority still varies with collective/inflow and must be measured.')
    add('FW_USE_AIRSPD',1,'Keep measured airspeed enabled for TECS, coordination and transition; only aerodynamic RATE output scaling is disabled.')
    add('FW_THR_MIN',.08,'Use the same minimum command as inherited MPC_THR_MIN to avoid zero-collective rotor control in the first FW test. This does not prove sufficient torque margin or define idle thrust.')
    add('VT_B_TRANS_DUR',15,'Trial 90deg/15s = 6deg/s back-rotation instead of prior 18deg/s. With unchanged MC pitch attitude P=0.58, an ideal rate-tracking screen gives ~10.4deg end-ramp lag versus ~15.5deg at 10s and a ~14.9deg exit threshold. Not a recovery prediction: timeout and residual speed still require evidence; longer travel must be allowed.')
    add('FW_RLL_TO_YAW_FF',0,'Keep the prior zero adverse-yaw torque feedforward; do not add unmeasured cross-axis feedforward during the first FW trial.')
    add("FW_THR_MAX", 1, "Use the physically power-limited engine envelope, not the old 9.6kW model's 50% cap.")
    add("VT_F_TRANS_THR", .4, "Retained prior value; the pinned tailsitter transition uses MC thrust, not this Standard-VTOL throttle parameter. It is not a tailsitter acceleration tune.")
    add("FW_THR_TRIM", .25, "Provisional level-flight drag/power seed, not a calibrated trim point.")
    for name,value in (('FW_AIRSPD_STALL',17),('FW_AIRSPD_MIN',21),('FW_AIRSPD_TRIM',26),
                       ('FW_AIRSPD_MAX',34),('VT_ARSP_BLEND',20),('VT_ARSP_TRANS',25)):
        reason=('Retained prior value; the pinned tailsitter has no Standard-VTOL airspeed blending schedule controlled by this parameter.' if name=='VT_ARSP_BLEND' else
                'Sizing-screen envelope at 2kg with CLmax1.0..1.2 uncertainty; actual stall/trim and transition lift must be measured.')
        add(name,value,reason)
    return params, reasons


def build(source: Path, output: Path, blade_chord_scale: float = 2.6) -> dict:
    if not math.isclose(blade_chord_scale, 2.6, abs_tol=1e-12):
        raise ValueError('This profile is screened only at chord scale 2.6. Recompute BEM seeds before changing it.')
    text = source.read_text(encoding="utf-8")
    values = parse_acf(source)
    replacements = replacements_for(values, blade_chord_scale)
    target = output / "aircraft" / NAME
    if target.exists():
        raise FileExistsError(f"Refusing to overwrite generated aircraft: {target}")
    target.mkdir(parents=True)
    acf = target / f"{NAME}.acf"
    acf.write_text(rewrite(text, replacements), encoding="utf-8")
    for path in source.parent.iterdir():
        if path.is_dir() and path.name == "Airfoil":
            shutil.copytree(path, target / path.name)
        elif path.suffix.lower() == ".png":
            shutil.copy2(path, target / path.name.replace("QuadTailsitter", NAME))
    generated_values = parse_acf(acf)
    params, reasons = parameter_profile(generated_values)
    profile = ["#!/bin/sh", "# SITL ONLY: experimental Ref6. Never install on real hardware."]
    for name, value in params.items():
        profile.extend((f"# {reasons[name]}", f"param set {name} {value:.9g}", f"param show {name}"))
    (output / "profile.sh").write_text("\n".join(profile) + "\n")
    (output / "parameters.json").write_text(json.dumps({"parameters": params, "reasons": reasons}, indent=2) + "\n")
    hubs = [(as_float(generated_values, f"_engn/{i}/_part_x") * FT_TO_M,
             as_float(generated_values, f"_engn/{i}/_part_z") * FT_TO_M) for i in range(4)]
    gap = min(math.dist(a, b) for i, a in enumerate(hubs) for b in hubs[i + 1:]) - DIAMETER_M
    if gap <= 0:
        raise ValueError("Propeller disks overlap")
    rho = 1.059
    tw = 4 * APC_STATIC[1][1] * rho / 1.225 / (MASS_KG * G)
    doc = {
        "name": NAME, "display_name": "Compact BSC3120 X-Tailsitter (2kg candidate)", "revision": "2026-11-02.ref6.9", "flight_validated": False,
        "source_sha256": hashlib.sha256(text.encode()).hexdigest(), "acf_sha256": sha256(acf),
        "mass_kg": MASS_KG, "main_panel_area_m2": MAIN_AREA_M2,
        "main_chord_scale": sizing.CHORD / (as_float(values, '_wing/0/_Croot') * FT_TO_M),
        "propeller_diameter_m": DIAMETER_M, "propeller_pitch_m": PITCH_M,
        "blade_chord_scale": blade_chord_scale, "rpm_limit": RPM_LIMIT,
        "shaft_power_budget_w": MOTOR_KW * 4000,
        "assumed_efficiency": EFFICIENCY, "auxiliary_power_w": AUX_W,
        "estimated_electrical_budget_w": MOTOR_KW * 4000 / EFFICIENCY + AUX_W,
        "battery_nominal_budget_w": as_float(values, "acf/_nom_bat_volt") * as_float(values, "acf/_max_bat_amp"),
        "minimum_planar_prop_clearance_m": gap,
        "apc_reference_tw_at_12000rpm_rho1p059": tw,
        "candidate_bem_tw_at_12000rpm_rho1p059": 4 * BEM_STATIC_THRUST_N * rho / 1.225 / (MASS_KG * G),
        "engine_free_rev_time_s": .1,
        "prop_material_density_assumption_kg_m3": 1400,
        "response_assumptions": "0.1s engine-only free rev and 1400kg/m3 composite proxy. Loaded response is NOT identified by these settings; measure RPM/thrust rise and fall.",
        "inertia": "Component tensor calculated in sizing.json; X-Plane automatic inertia retained because serialized manual-inertia units are unverified. Runtime comparison is a release gate, not claimed completed.",
        "cg_xplane_m": sizing.screen()['cg_xplane_m'],
        "motor_model": "MAD BSC3120 1000KV, 6S reference; 10x8 prop combination not manufacturer tested",
        "acf_power_max_limit_units": "horsepower per Plane Maker manual; 500W/745.699872 serialized, not kW",
        "geometry_tradeoff": "Inherited secondary panels disabled to avoid uncontrolled trim contributions; complete-aircraft neutral point still requires measurement.",
        "direction_contract": "Original ACF engine/blade orientations and propeller rotation signs retained, same channel-to-engine mapping and quadrant ordering. Allocation direction vectors from pinned official airframe. No new axis inversion introduced.",
        "propulsion_calibration": "PENDING X-Plane measurement. Geometric 10x8 candidate, NOT an APC replica or calibrated model.",
        "bem_screening": {"rpm": BEM_STATIC_RPM, "thrust_n": BEM_STATIC_THRUST_N, "shaft_power_w": BEM_STATIC_POWER_W,
                          "valid_for_blade_chord_scale": 2.6, "uncertainty_allowance_fraction": .2,
                          "note": "First supplied NACA2412 polar; see bundle PROPULSION_REVIEW.md. Recompute for other chord scales."},
        "sources": [APC_URL, "https://www.apcprop.com/technical-information/rpm-limits/", "https://developer.x-plane.com/manuals/planemaker/"],
        "acf_changes": {key: {"before": values[key], "after": value} for key, value in replacements.items() if "_geo_xyz" not in key},
        "release_gates": ["actual mass and CG", "allocation force/moment signs", "actual RPM ceiling and shaft power", "static thrust and Q/T", "loaded acceleration/deceleration", "hover", "FW rate tracking", "back-transition", "Land descent and disarm"],
    }
    (output / "design.json").write_text(json.dumps(doc, indent=2) + "\n")
    (output / "sizing.json").write_text(json.dumps(sizing.screen(), indent=2)+'\n')
    return doc


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--blade-chord-scale", type=float, default=2.6)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output, args.blade_chord_scale), indent=2))
