#!/usr/bin/env python3
"""Summarize QuadTailsitter X-Plane aircraft sizing and optional truth data."""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import statistics
import zipfile
from pathlib import Path
from typing import Dict, Iterable, List, Optional, TextIO


FT_TO_M = 0.3048
LB_TO_KG = 0.45359237
KNOT_TO_MPS = 0.514444
G = 9.80665
RHO0 = 1.225
SPEED_OF_SOUND = 343.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("acf", help="QuadTailsitter .acf file")
    parser.add_argument("--truth", help="Optional XPlaneTruthCapture folder or zip")
    parser.add_argument("--clmax", type=float, default=1.2, help="Reference CLmax for stall estimate")
    parser.add_argument("--json", action="store_true", dest="json_output", help="Emit JSON")
    return parser.parse_args()


def parse_acf(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[0] == "P":
            values[parts[1]] = parts[2]
    return values


def as_float(values: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(values.get(key, default))
    except (TypeError, ValueError):
        return default


def percentile(values: Iterable[float], fraction: float) -> Optional[float]:
    numbers = sorted(v for v in values if math.isfinite(v))
    if not numbers:
        return None
    index = int(round((len(numbers) - 1) * max(0.0, min(1.0, fraction))))
    return numbers[index]


def stats(values: Iterable[float]) -> Optional[Dict[str, float]]:
    numbers = [v for v in values if math.isfinite(v)]
    if not numbers:
        return None
    return {
        "min": min(numbers),
        "p50": percentile(numbers, 0.50) or 0.0,
        "p95": percentile(numbers, 0.95) or 0.0,
        "max": max(numbers),
        "mean": statistics.fmean(numbers),
    }


def parse_float_array(value: str) -> List[float]:
    if not value:
        return []
    result: List[float] = []
    for item in value.split(";"):
        try:
            result.append(float(item))
        except ValueError:
            pass
    return result


class CaptureFrames:
    def __init__(self, capture: Path):
        self.capture = capture
        self._zip: Optional[zipfile.ZipFile] = None
        self._text: Optional[TextIO] = None

    def __enter__(self) -> TextIO:
        if self.capture.is_file() and self.capture.suffix.lower() == ".zip":
            self._zip = zipfile.ZipFile(self.capture)
            candidates = [name for name in self._zip.namelist() if name.endswith("/frames.csv") or name == "frames.csv"]
            if not candidates:
                raise FileNotFoundError(f"frames.csv not found in {self.capture}")
            self._text = io.TextIOWrapper(self._zip.open(candidates[0], "r"), encoding="utf-8", newline="")
            return self._text

        frames_path = self.capture / "frames.csv"
        if not frames_path.exists():
            nested = list(self.capture.glob("*/frames.csv"))
            if len(nested) == 1:
                frames_path = nested[0]
        if not frames_path.exists():
            raise FileNotFoundError(f"frames.csv not found under {self.capture}")
        self._text = frames_path.open("r", encoding="utf-8", newline="")
        return self._text

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._text is not None:
            self._text.close()
        if self._zip is not None:
            self._zip.close()


def estimate_wing_area_m2(values: Dict[str, str]) -> Dict[str, float]:
    raw_area_ft2 = 0.0
    mirrored_area_ft2 = 0.0
    wing_count = int(as_float(values, "_wing/count", 0))
    for index in range(wing_count):
        semilen = as_float(values, f"_wing/{index}/_semilen_SEG")
        croot = as_float(values, f"_wing/{index}/_Croot")
        ctip = as_float(values, f"_wing/{index}/_Ctip")
        if semilen <= 0.0 or croot <= 0.0 or ctip <= 0.0:
            continue
        area_ft2 = semilen * (croot + ctip) * 0.5
        raw_area_ft2 += area_ft2
        right_multiplier = as_float(values, f"_wing/{index}/_is_right_mult", 0.0)
        mirrored_area_ft2 += area_ft2 * (2.0 if right_multiplier > 0.0 else 1.0)
    return {
        "raw_area_m2": raw_area_ft2 * FT_TO_M * FT_TO_M,
        "mirrored_area_m2": mirrored_area_ft2 * FT_TO_M * FT_TO_M,
    }


def aircraft_summary(values: Dict[str, str], clmax: float) -> Dict[str, object]:
    empty_lb = as_float(values, "acf/_m_empty")
    max_lb = as_float(values, "acf/_m_max")
    mass_kg = empty_lb * LB_TO_KG
    weight_n = mass_kg * G
    areas = estimate_wing_area_m2(values)
    area_m2 = areas["mirrored_area_m2"] or areas["raw_area_m2"]
    wing_loading = weight_n / area_m2 if area_m2 > 0.0 else 0.0
    stall_mps = math.sqrt((2.0 * weight_n) / (RHO0 * area_m2 * clmax)) if area_m2 > 0.0 and clmax > 0.0 else 0.0

    engines = []
    props = []
    total_disk_area_m2 = 0.0
    for index in range(4):
        prop_radius_m = as_float(values, f"_blad/{index}/_semilen_SEG") * FT_TO_M
        total_disk_area_m2 += math.pi * prop_radius_m * prop_radius_m
        design_rpm = as_float(values, f"_prop/{index}/_des_rpm_prp")
        redline_rpm = as_float(values, f"_engn/{index}/_RSC_redline_ENGN")
        props.append(
            {
                "index": index,
                "diameter_in": prop_radius_m * 2.0 / 0.0254,
                "blades": as_float(values, f"_prop/{index}/_num_blades"),
                "design_rpm": design_rpm,
                "design_tip_mach": (2.0 * math.pi * prop_radius_m * design_rpm / 60.0) / SPEED_OF_SOUND,
                "redline_tip_mach": (2.0 * math.pi * prop_radius_m * redline_rpm / 60.0) / SPEED_OF_SOUND,
                "design_speed_mps": as_float(values, f"_prop/{index}/_des_kts_acf") * KNOT_TO_MPS,
                "direction": as_float(values, f"_prop/{index}/_prop_dir"),
            }
        )
        engines.append(
            {
                "index": index,
                "x_ft": as_float(values, f"_engn/{index}/_part_x"),
                "y_ft": as_float(values, f"_engn/{index}/_part_y"),
                "z_ft": as_float(values, f"_engn/{index}/_part_z"),
                "yaw_deg": as_float(values, f"_engn/{index}/_part_psi"),
                "pitch_deg": as_float(values, f"_engn/{index}/_part_the"),
                "redline_rpm": redline_rpm,
                "power_max_kw": as_float(values, f"_engn/{index}/_power_max_limit"),
            }
        )

    return {
        "mass": {
            "empty_lb": empty_lb,
            "empty_kg": mass_kg,
            "max_lb": max_lb,
            "max_kg": max_lb * LB_TO_KG,
            "weight_n": weight_n,
        },
        "battery": {
            "voltage": as_float(values, "acf/_nom_bat_volt"),
            "watt_hours": as_float(values, "acf/_battery_watt_hr_max"),
            "max_amps": as_float(values, "acf/_max_bat_amp"),
            "max_kw": as_float(values, "acf/_nom_bat_volt") * as_float(values, "acf/_max_bat_amp") / 1000.0,
        },
        "wing": {
            **areas,
            "selected_area_m2": area_m2,
            "wing_loading_n_m2": wing_loading,
            "wing_loading_kg_m2": mass_kg / area_m2 if area_m2 > 0.0 else 0.0,
            "stall_mps_at_clmax": stall_mps,
            "stall_kts_at_clmax": stall_mps / KNOT_TO_MPS if stall_mps > 0.0 else 0.0,
            "clmax": clmax,
        },
        "propulsion": {
            "hover_total_thrust_n": weight_n,
            "hover_per_motor_thrust_n": weight_n / 4.0,
            "target_total_static_thrust_n_1p5_tw": weight_n * 1.5,
            "target_per_motor_static_thrust_n_1p5_tw": weight_n * 1.5 / 4.0,
            "disk_loading_n_m2": weight_n / total_disk_area_m2 if total_disk_area_m2 > 0.0 else 0.0,
            "engines": engines,
            "props": props,
        },
    }


def truth_summary(capture: Path) -> Dict[str, object]:
    values: Dict[str, List[float]] = {
        "mass_kg": [],
        "true_airspeed_mps": [],
        "indicated_airspeed_kt": [],
        "total_thrust_n": [],
        "max_prop_rpm": [],
        "max_throttle_used": [],
    }
    with CaptureFrames(capture) as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                values["mass_kg"].append(float(row.get("sim/flightmodel/weight/m_total", "")))
            except ValueError:
                pass
            for key, column in (
                ("true_airspeed_mps", "sim/flightmodel/position/true_airspeed"),
                ("indicated_airspeed_kt", "sim/flightmodel/position/indicated_airspeed"),
            ):
                try:
                    values[key].append(float(row.get(column, "")))
                except ValueError:
                    pass
            thrust = parse_float_array(row.get("sim/flightmodel/engine/POINT_thrust", ""))
            if thrust:
                values["total_thrust_n"].append(sum(thrust[:4]))
            rpm = parse_float_array(row.get("sim/flightmodel/engine/ENGN_rpm", ""))
            if rpm:
                values["max_prop_rpm"].append(max(abs(v) for v in rpm[:4]))
            throttle = parse_float_array(row.get("sim/flightmodel2/engines/throttle_used_ratio", ""))
            if throttle:
                values["max_throttle_used"].append(max(throttle[:4]))

    return {name: stats(series) for name, series in values.items() if stats(series)}


def analyze(acf: Path, truth: Optional[Path], clmax: float) -> Dict[str, object]:
    values = parse_acf(acf)
    result: Dict[str, object] = {
        "acf": str(acf),
        "aircraft": aircraft_summary(values, clmax),
    }
    if truth:
        result["truth"] = truth_summary(truth)
    return result


def print_text(result: Dict[str, object]) -> None:
    aircraft = result["aircraft"]  # type: ignore[index]
    mass = aircraft["mass"]  # type: ignore[index]
    wing = aircraft["wing"]  # type: ignore[index]
    propulsion = aircraft["propulsion"]  # type: ignore[index]
    battery = aircraft["battery"]  # type: ignore[index]

    print(f"ACF: {result['acf']}")
    print(f"Mass: {mass['empty_kg']:.2f} kg empty ({mass['empty_lb']:.2f} lb), max {mass['max_kg']:.2f} kg")
    print(f"Battery: {battery['voltage']:.1f} V, {battery['watt_hours']:.0f} Wh, {battery['max_amps']:.0f} A ({battery['max_kw']:.1f} kW)")
    print(f"Wing area estimate: {wing['selected_area_m2']:.3f} m^2, loading {wing['wing_loading_n_m2']:.1f} N/m^2")
    print(f"Stall estimate: {wing['stall_mps_at_clmax']:.1f} m/s at CLmax={wing['clmax']:.2f}")
    print(f"Hover thrust: {propulsion['hover_total_thrust_n']:.1f} N total, {propulsion['hover_per_motor_thrust_n']:.1f} N per motor")
    print(f"1.5 T/W target: {propulsion['target_total_static_thrust_n_1p5_tw']:.1f} N total")
    for prop in propulsion["props"]:  # type: ignore[index]
        print(
            f"Prop {prop['index']}: {prop['diameter_in']:.1f} in, {prop['blades']:.0f} blades, "
            f"design {prop['design_rpm']:.0f} rpm Mach {prop['design_tip_mach']:.2f}, "
            f"redline Mach {prop['redline_tip_mach']:.2f}"
        )
    if "truth" in result:
        print("\nTruthCapture:")
        for key, value in result["truth"].items():  # type: ignore[union-attr]
            print(f"  {key}: {value}")


def main() -> int:
    args = parse_args()
    result = analyze(Path(args.acf), Path(args.truth) if args.truth else None, args.clmax)
    if args.json_output:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_text(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
