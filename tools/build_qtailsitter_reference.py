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

NAME = "QuadTailsitterRef6"
MASS_KG = 6.0
MAIN_AREA_M2 = 0.331
DIAMETER_M = 0.254
PITCH_M = 0.2032
RPM_LIMIT = 13000
MOTOR_KW = 0.75
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
    ratio = MAIN_AREA_M2 / area
    result = {
        "acf/_m_empty": number(MASS_KG / LB_TO_KG),
        "acf/_m_max": number(MASS_KG / LB_TO_KG),
        "acf/_fixed_max/0": number(0),
        "acf/_engine_rev_time_sec": number(0.1),
        "acf/_rotor_mi_rat": number(1400 / 2700),
        "acf/_fadec_EQ_rpm_limit": "1",
        "acf/_use_manual_prop_inc": "1",
    }
    for index in range(4):
        wing = f"_wing/{index}"
        for key in ("_Croot", "_Ctip"):
            result[f"{wing}/{key}"] = number(as_float(values, f"{wing}/{key}") * ratio)
        for rib in range(int(as_float(values, f"{wing}/_chord_for_RIB/count"))):
            key = f"{wing}/_chord_for_RIB/{rib}"
            result[key] = number(as_float(values, key) * ratio)
        # Preserve the quarter-chord line and panel sweep. Stored mesh uses
        # absolute aircraft coordinates (feet), with chord along Y at 90 deg.
        px = as_float(values, f"{wing}/_part_x")
        py = as_float(values, f"{wing}/_part_y")
        pz = as_float(values, f"{wing}/_part_z")
        sweep = math.radians(as_float(values, f"{wing}/_sweep_design"))
        for key, value in values.items():
            match = re.fullmatch(re.escape(wing) + r"/_geo_xyz/(\d+),(\d+),([012])", key)
            if not match:
                continue
            station, section, axis = map(int, match.groups())
            if station >= int(as_float(values, f"{wing}/_s_dim")) or section >= int(as_float(values, f"{wing}/_r_dim")):
                continue
            if axis == 1:
                result[key] = number(py + (float(value) - py) * ratio)
            elif axis == 2:
                x = as_float(values, f"{wing}/_geo_xyz/{station},{section},0")
                center_z = pz + abs(x - px) * math.tan(sweep)
                result[key] = number(center_z + (float(value) - center_z) * ratio)

        engine = f"_engn/{index}"
        result[f"{engine}/_power_max_limit"] = number(MOTOR_KW)
        result[f"{engine}/_RSC_redline_ENGN"] = number(RPM_LIMIT)
        result[f"{engine}/_RSC_max_pwr_ENGN"] = number(RPM_LIMIT)
        # A deliberately uncanted reference avoids inheriting an unverified
        # compound cant convention. Allocation uses the same vertical axes.
        result[f"{engine}/_part_psi"] = number(0)
        result[f"{engine}/_part_the"] = number(90)
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
    return result


def parameter_profile(values: dict[str, str]) -> tuple[dict[str, float], dict[str, str]]:
    params = {}
    reasons = {}
    def add(name: str, value: float, reason: str):
        params[name] = value
        reasons[name] = reason

    # The controller consumes normalized thrust, not shaft-power fraction.
    # Ref6 flight 2026-11-01, boot 138..155s: median commanded hover thrust
    # 0.7107 at 6kg. The original power-fraction seed delayed liftoff and
    # saturated the estimator's allowed adaptation range.
    add("MPC_THR_HOVER", .71, "Measured Ref6 normalized hover thrust, boot 138..155s in flight-8yc79oye. Estimator remains enabled; not a universal airframe value.")
    for index in range(4):
        engine = f"_engn/{index}"
        add(f"CA_ROTOR{index}_PX", -as_float(values, f"{engine}/_part_z") * FT_TO_M, "Actual ACF hub position in MC FRD, meters.")
        add(f"CA_ROTOR{index}_PY", as_float(values, f"{engine}/_part_x") * FT_TO_M, "Actual ACF hub position in MC FRD, meters.")
        add(f"CA_ROTOR{index}_PZ", -(as_float(values, f"{engine}/_part_y") - as_float(values, "acf/_cgY")) * FT_TO_M, "Actual hub height relative to CG.")
        for axis, value in (("AX", 0), ("AY", 0), ("AZ", -1)):
            add(f"CA_ROTOR{index}_{axis}", value, "Uncanted reference; verify actual force axes before flight.")
        direction = as_float(values, f"_prop/{index}/_prop_dir")
        add(f"CA_ROTOR{index}_CT", 6.5, "PX4 default common allocation scale. CT=2 with this KM fell below the allocator's 0.05 weak-axis cutoff. Common scaling is removed by normalized allocation; not a measured maximum thrust.")
        km = BEM_STATIC_POWER_W / (BEM_STATIC_RPM * math.pi / 30) / BEM_STATIC_THRUST_N
        add(f"CA_ROTOR{index}_KM", direction * km, "BEM static Q/T prediction; rotation sign and actual Q/T must pass bench test.")
    # Restore the known quiet-hover gains, rather than transfer increased gains
    # from a propulsion system with different dynamics. Other stock 5021 gains
    # remain visible in the captured airframe and parameter export.
    for name, value in (("MC_ROLLRATE_P", .10), ("MC_PITCHRATE_P", .115),
                        ("MC_ROLLRATE_D", .0008), ("FW_RR_P", .10)):
        add(name, value, "Known quiet-hover baseline seed; not a claim of a tuned Ref6 controller.")
    add("FW_THR_MAX", 1, "Use the physically power-limited engine envelope, not the old 9.6kW model's 50% cap.")
    add("VT_F_TRANS_THR", .55, "Provisional 1.65kW aggregate shaft-power request; validate acceleration before FW release.")
    add("FW_THR_TRIM", .30, "Provisional trim seed (900W shaft); measure drag/power in level flight before optimization.")
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
    params, reasons = parameter_profile(values)
    profile = ["#!/bin/sh", "# SITL ONLY: experimental Ref6. Never install on real hardware."]
    for name, value in params.items():
        profile.extend((f"# {reasons[name]}", f"param set {name} {value:.9g}", f"param show {name}"))
    (output / "profile.sh").write_text("\n".join(profile) + "\n")
    (output / "parameters.json").write_text(json.dumps({"parameters": params, "reasons": reasons}, indent=2) + "\n")
    hubs = [(as_float(values, f"_engn/{i}/_part_x") * FT_TO_M,
             as_float(values, f"_engn/{i}/_part_z") * FT_TO_M) for i in range(4)]
    gap = min(math.dist(a, b) for i, a in enumerate(hubs) for b in hubs[i + 1:]) - DIAMETER_M
    if gap <= 0:
        raise ValueError("Propeller disks overlap")
    rho = 1.059
    tw = 4 * APC_STATIC[1][1] * rho / 1.225 / (MASS_KG * G)
    doc = {
        "name": NAME, "revision": "2026-11-01.ref6.4", "flight_validated": False,
        "source_sha256": hashlib.sha256(text.encode()).hexdigest(), "acf_sha256": sha256(acf),
        "mass_kg": MASS_KG, "main_panel_area_m2": MAIN_AREA_M2,
        "main_chord_scale": MAIN_AREA_M2 / estimate_wing_area_m2(values)["main_panel_area_m2"],
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
        "inertia": "X-Plane automatic inertia retained; runtime verification required. Zero manual fields do not mean zero inertia.",
        "propulsion_calibration": "PENDING X-Plane measurement. Geometric 10x8 candidate, NOT an APC replica or calibrated model.",
        "bem_screening": {"rpm": BEM_STATIC_RPM, "thrust_n": BEM_STATIC_THRUST_N, "shaft_power_w": BEM_STATIC_POWER_W,
                          "valid_for_blade_chord_scale": 2.6, "uncertainty_allowance_fraction": .2,
                          "note": "First supplied NACA2412 polar; see bundle PROPULSION_REVIEW.md. Recompute for other chord scales."},
        "sources": [APC_URL, "https://www.apcprop.com/technical-information/rpm-limits/", "https://developer.x-plane.com/manuals/planemaker/"],
        "acf_changes": {key: {"before": values[key], "after": value} for key, value in replacements.items() if "_geo_xyz" not in key},
        "release_gates": ["actual mass and CG", "allocation force/moment signs", "actual RPM ceiling and shaft power", "static thrust and Q/T", "loaded acceleration/deceleration", "hover", "FW rate tracking", "back-transition", "Land descent and disarm"],
    }
    (output / "design.json").write_text(json.dumps(doc, indent=2) + "\n")
    return doc


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--blade-chord-scale", type=float, default=2.6)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output, args.blade_chord_scale), indent=2))
