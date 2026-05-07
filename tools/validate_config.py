#!/usr/bin/env python3
"""Validate px4xplane airframe actuator mappings in config.ini."""

from __future__ import annotations

import argparse
import configparser
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


RANGE_RE = re.compile(r"^\[\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*\]$")
INDEX_RE = re.compile(r"^\[\s*(\d+(?:\s+\d+)*)\s*\]$")
SUPPORTED_TYPES = {"float", "floatArray"}
GLOBAL_SECTION = "__px4xplane_global__"


@dataclass
class Issue:
    level: str
    section: str
    key: str
    message: str

    def __str__(self) -> str:
        location = self.section if not self.key else f"{self.section}.{self.key}"
        return f"{self.level}: {location}: {self.message}"


def split_parts(value: str) -> list[str]:
    return [part.strip() for part in value.split(",")]


def parse_range(token: str) -> tuple[float, float] | None:
    match = RANGE_RE.match(token.strip())
    if not match:
        return None
    low = float(match.group(1))
    high = float(match.group(2))
    if not math.isfinite(low) or not math.isfinite(high) or low == high:
        return None
    return low, high


def parse_indices(token: str) -> list[int] | None:
    token = token.strip()
    if token == "0":
        return []
    match = INDEX_RE.match(token)
    if not match:
        return None
    return [int(item) for item in match.group(1).split()]


def validate_channel(section: str, key: str, value: str) -> Iterable[Issue]:
    mappings = [mapping.strip() for mapping in value.split("|") if mapping.strip()]
    if not mappings:
        yield Issue("error", section, key, "channel has no mappings")
        return

    for mapping_index, mapping in enumerate(mappings):
        parts = split_parts(mapping)
        label = f"{key}[{mapping_index}]"
        if len(parts) != 4:
            yield Issue("error", section, label, "expected 4 comma-separated fields: dataref, type, indices, range")
            continue

        dataref, data_type, indices_token, range_token = parts
        if not dataref:
            yield Issue("error", section, label, "empty dataref name")
        if data_type not in SUPPORTED_TYPES:
            yield Issue("error", section, label, f"unsupported type '{data_type}'")

        indices = parse_indices(indices_token)
        if indices is None:
            yield Issue("error", section, label, f"invalid array index token '{indices_token}'")
        elif data_type == "float" and indices:
            yield Issue("warning", section, label, "float mapping should not specify array indices")
        elif data_type == "floatArray" and not indices:
            yield Issue("error", section, label, "floatArray mapping must specify at least one array index")
        elif any(index < 0 for index in indices):
            yield Issue("error", section, label, "array indices must be non-negative")

        if parse_range(range_token) is None:
            yield Issue("error", section, label, f"invalid range '{range_token}'")


def validate_config(path: Path) -> list[Issue]:
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    parser.optionxform = str
    text = path.read_text(encoding="utf-8-sig")
    parser.read_string(f"[{GLOBAL_SECTION}]\n{text}", source=str(path))

    issues: list[Issue] = []
    active = parser.get(GLOBAL_SECTION, "config_name", fallback="").strip()
    if not active:
        issues.append(Issue("error", "global", "config_name", "missing active config_name"))
    elif active not in parser.sections():
        issues.append(Issue("error", "global", "config_name", f"active config '{active}' has no matching section"))

    for section in parser.sections():
        if section == GLOBAL_SECTION:
            continue
        seen_channels: set[int] = set()
        for key, value in parser.items(section, raw=True):
            if not key.startswith("channel"):
                continue
            suffix = key.removeprefix("channel")
            if not suffix.isdigit():
                issues.append(Issue("error", section, key, "channel key must be channel0..channel15"))
                continue
            channel = int(suffix)
            seen_channels.add(channel)
            if channel < 0 or channel > 15:
                issues.append(Issue("error", section, key, "channel number must be in 0..15"))
                continue
            issues.extend(validate_channel(section, key, value))

        if section == active and not seen_channels:
            issues.append(Issue("error", section, "", "active airframe has no channel mappings"))

        brakes = parser.get(section, "autoPropBrakes", fallback="").strip()
        if brakes:
            for token in brakes.split(","):
                token = token.strip()
                if not token:
                    continue
                if not token.isdigit():
                    issues.append(Issue("error", section, "autoPropBrakes", f"invalid motor index '{token}'"))
                    continue
                index = int(token)
                if index < 0 or index > 7:
                    issues.append(Issue("error", section, "autoPropBrakes", f"motor index {index} outside 0..7"))

    return issues


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", default="config/config.ini", help="Path to px4xplane config.ini")
    parser.add_argument("--warnings-as-errors", action="store_true", help="Return failure when warnings are present")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    issues = validate_config(Path(args.config))
    for issue in issues:
        print(issue)

    has_errors = any(issue.level == "error" for issue in issues)
    has_warnings = any(issue.level == "warning" for issue in issues)
    if has_errors or (args.warnings_as_errors and has_warnings):
        return 1

    print(f"OK: {args.config}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
