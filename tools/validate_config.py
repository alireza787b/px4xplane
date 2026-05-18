#!/usr/bin/env python3
"""Validate px4xplane airframe actuator mappings in config.ini."""

from __future__ import annotations

import argparse
import configparser
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


RANGE_RE = re.compile(r"^\[\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*\]$")
INDEX_RE = re.compile(r"^\[\s*(\d+(?:\s+\d+)*)\s*\]$")
SUPPORTED_TYPES = {"float", "floatArray"}
GLOBAL_SECTION = "__px4xplane_global__"
DEFAULT_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "config" / "config_schema.json"


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


def load_schema(path: Path = DEFAULT_SCHEMA_PATH) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_bool(value: str) -> bool | None:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    return None


def validate_scalar_field(section: str, key: str, value: str, field_schema: dict[str, Any]) -> Iterable[Issue]:
    value = value.strip()
    field_type = field_schema.get("type")

    if field_schema.get("required") and not value:
        yield Issue("error", section, key, "required value is empty")
        return

    if field_type == "string":
        return

    if field_type == "bool":
        if parse_bool(value) is None:
            yield Issue("error", section, key, f"expected boolean, got '{value}'")
        return

    if field_type == "int":
        try:
            parsed = int(value)
        except ValueError:
            yield Issue("error", section, key, f"expected integer, got '{value}'")
            return
        yield from validate_numeric_range(section, key, float(parsed), field_schema)
        return

    if field_type == "float":
        try:
            parsed = float(value)
        except ValueError:
            yield Issue("error", section, key, f"expected float, got '{value}'")
            return
        if not math.isfinite(parsed):
            yield Issue("error", section, key, f"expected finite float, got '{value}'")
            return
        yield from validate_numeric_range(section, key, parsed, field_schema)
        return

    yield Issue("warning", section, key, f"schema type '{field_type}' is not validated by this tool")


def validate_numeric_range(section: str, key: str, value: float, field_schema: dict[str, Any]) -> Iterable[Issue]:
    minimum = field_schema.get("min")
    maximum = field_schema.get("max")
    if minimum is not None and value < float(minimum):
        yield Issue("error", section, key, f"value {value:g} is below minimum {minimum}")
    if maximum is not None and value > float(maximum):
        yield Issue("error", section, key, f"value {value:g} is above maximum {maximum}")


def validate_global_fields(parser: configparser.ConfigParser, schema: dict[str, Any]) -> Iterable[Issue]:
    field_schemas: dict[str, Any] = schema.get("global_fields", {})
    global_items = dict(parser.items(GLOBAL_SECTION, raw=True))

    for key, field_schema in field_schemas.items():
        if field_schema.get("required") and key not in global_items:
            yield Issue("error", "global", key, "missing required global field")

    for key, value in global_items.items():
        field_schema = field_schemas.get(key)
        if field_schema is None:
            yield Issue("warning", "global", key, "unknown global config key")
            continue
        yield from validate_scalar_field("global", key, value, field_schema)


def validate_channel(section: str, key: str, value: str, supported_types: set[str]) -> Iterable[Issue]:
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
        if data_type not in supported_types:
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


def validate_config(path: Path, schema_path: Path = DEFAULT_SCHEMA_PATH) -> list[Issue]:
    schema = load_schema(schema_path)
    supported_types = set(schema.get("runtime_supported_channel_types", sorted(SUPPORTED_TYPES)))
    airframe_field_schemas: dict[str, Any] = schema.get("airframe_fields", {})

    parser = configparser.ConfigParser(interpolation=None, strict=False)
    parser.optionxform = str
    text = path.read_text(encoding="utf-8-sig")
    parser.read_string(f"[{GLOBAL_SECTION}]\n{text}", source=str(path))

    issues: list[Issue] = []
    issues.extend(validate_global_fields(parser, schema))

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
            issues.extend(validate_channel(section, key, value, supported_types))

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

        for key, value in parser.items(section, raw=True):
            if key == "autoPropBrakes" or key.startswith("channel"):
                continue

            field_schema = airframe_field_schemas.get(key)
            if field_schema is None:
                issues.append(Issue("warning", section, key, "unknown airframe config key"))
                continue
            yield_issues = validate_scalar_field(section, key, value, field_schema)
            issues.extend(yield_issues)

        apply_key = "autoPropBrakeApplyThreshold"
        release_key = "autoPropBrakeReleaseThreshold"
        if parser.has_option(section, apply_key) and parser.has_option(section, release_key):
            try:
                apply_value = float(parser.get(section, apply_key, raw=True))
                release_value = float(parser.get(section, release_key, raw=True))
                if math.isfinite(apply_value) and math.isfinite(release_value) and release_value <= apply_value:
                    issues.append(Issue("error", section, release_key, "release threshold must be greater than apply threshold"))
            except ValueError:
                pass

    return issues


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", default="config/config.ini", help="Path to px4xplane config.ini")
    parser.add_argument("--schema", default=str(DEFAULT_SCHEMA_PATH), help="Path to px4xplane config schema JSON")
    parser.add_argument("--warnings-as-errors", action="store_true", help="Return failure when warnings are present")
    parser.add_argument("--list-fields", action="store_true", help="List schema-backed global fields and reload policies")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    schema_path = Path(args.schema)

    if args.list_fields:
        schema = load_schema(schema_path)
        for key, field in schema.get("global_fields", {}).items():
            field_type = field.get("type", "unknown")
            reload_policy = field.get("reload_policy", "unspecified")
            print(f"{key}: {field_type}, {reload_policy}")
        return 0

    issues = validate_config(Path(args.config), schema_path)
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
