#!/usr/bin/env python3
"""Check PX4 ULog initial parameters against an airframe default file."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple


SET_DEFAULT_RE = re.compile(r"^\s*param\s+set-default\s+([A-Za-z0-9_]+)\s+([^#\s]+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", help="PX4 .ulg file to inspect")
    parser.add_argument("airframe", help="PX4 airframe file containing param set-default lines")
    parser.add_argument("--param", action="append", dest="params", help="Limit check to one parameter name; repeatable")
    parser.add_argument("--abs-tol", type=float, default=1e-4, help="Absolute numeric tolerance")
    parser.add_argument("--rel-tol", type=float, default=1e-4, help="Relative numeric tolerance")
    parser.add_argument("--json", action="store_true", dest="json_output", help="Emit JSON")
    return parser.parse_args()


def parse_airframe_defaults(path: Path) -> Dict[str, str]:
    defaults: Dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = SET_DEFAULT_RE.match(line)
        if match:
            defaults[match.group(1)] = match.group(2)
    return defaults


def load_ulog_initial_parameters(path: Path) -> Dict[str, Any]:
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError("pyulog is not installed. Install with `python3 -m pip install pyulog`.") from exc

    ulog = ULog(str(path), [])
    return dict(getattr(ulog, "initial_parameters", {}))


def parse_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def values_match(expected: str, actual: Any, abs_tol: float, rel_tol: float) -> bool:
    expected_num = parse_number(expected)
    actual_num = parse_number(actual)
    if expected_num is None or actual_num is None:
        return str(expected) == str(actual)
    return math.isclose(actual_num, expected_num, abs_tol=abs_tol, rel_tol=rel_tol)


def compare_parameters(
    expected: Dict[str, str],
    actual: Dict[str, Any],
    selected: Iterable[str] | None,
    abs_tol: float,
    rel_tol: float,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    names = sorted(set(selected) if selected else expected.keys())
    matches: List[Dict[str, Any]] = []
    mismatches: List[Dict[str, Any]] = []
    for name in names:
        if name not in expected:
            mismatches.append({"param": name, "expected": None, "actual": actual.get(name), "status": "not_in_airframe"})
            continue
        if name not in actual:
            mismatches.append({"param": name, "expected": expected[name], "actual": None, "status": "missing_in_ulog"})
            continue
        entry = {"param": name, "expected": expected[name], "actual": actual[name]}
        if values_match(expected[name], actual[name], abs_tol, rel_tol):
            matches.append({**entry, "status": "match"})
        else:
            mismatches.append({**entry, "status": "mismatch"})
    return matches, mismatches


def check(ulog: Path, airframe: Path, params: Iterable[str] | None, abs_tol: float, rel_tol: float) -> Dict[str, Any]:
    expected = parse_airframe_defaults(airframe)
    actual = load_ulog_initial_parameters(ulog)
    matches, mismatches = compare_parameters(expected, actual, params, abs_tol, rel_tol)
    return {
        "ulog": str(ulog),
        "airframe": str(airframe),
        "checked": len(matches) + len(mismatches),
        "matched": len(matches),
        "mismatched": len(mismatches),
        "matches": matches,
        "mismatches": mismatches,
    }


def print_text(result: Dict[str, Any]) -> None:
    print(f"ULog: {result['ulog']}")
    print(f"Airframe: {result['airframe']}")
    print(f"Checked: {result['checked']}  matched: {result['matched']}  mismatched: {result['mismatched']}")
    if result["mismatches"]:
        print("\nMismatches:")
        for item in result["mismatches"]:
            print(f"  {item['param']}: expected {item['expected']} actual {item['actual']} ({item['status']})")


def main() -> int:
    args = parse_args()
    try:
        result = check(
            Path(args.ulog),
            Path(args.airframe),
            args.params,
            args.abs_tol,
            args.rel_tol,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.json_output:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_text(result)
    return 1 if result["mismatches"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
