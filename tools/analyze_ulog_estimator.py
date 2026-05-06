#!/usr/bin/env python3
"""Summarize PX4 ULog estimator topics for px4xplane validation runs."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


TOPICS = [
    "estimator_status",
    "estimator_status_flags",
    "estimator_innovation_test_ratios",
    "estimator_aid_src_baro_hgt",
    "estimator_aid_src_gnss_vel",
    "vehicle_local_position",
    "vehicle_air_data",
    "sensor_baro",
    "vehicle_gps_position",
    "commander_state",
]

WINDOW_FIELDS = {
    "vehicle_local_position": ["z", "vz", "z_deriv", "dist_bottom"],
    "estimator_aid_src_baro_hgt": ["test_ratio", "innovation", "innovation_variance", "timestamp_sample"],
    "estimator_aid_src_gnss_vel": ["test_ratio", "innovation", "innovation_variance", "timestamp_sample"],
    "estimator_innovation_test_ratios": ["vel_test_ratio", "pos_test_ratio", "hgt_test_ratio", "tas_test_ratio", "hagl_test_ratio"],
    "vehicle_air_data": ["baro_alt_meter", "baro_temp_celcius", "baro_pressure_pa", "rho"],
    "sensor_baro": ["pressure", "temperature", "error_count", "device_id"],
    "vehicle_gps_position": ["alt", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "eph", "epv", "s_variance_m_s"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", help="PX4 .ulg file")
    parser.add_argument("--json", dest="json_output", action="store_true", help="Emit JSON instead of text")
    parser.add_argument("--event-window-s", type=float, default=15.0, help="Seconds around logged warning events")
    return parser.parse_args()


def load_ulog(path: Path):
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError("pyulog is not installed. Install with `python3 -m pip install pyulog`.") from exc

    return ULog(str(path), TOPICS)


def as_list(values: Any) -> List[Any]:
    try:
        return list(values)
    except TypeError:
        return []


def finite_numbers(values: Iterable[Any]) -> List[float]:
    numbers: List[float] = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            numbers.append(number)
    return numbers


def basic_stats(values: Iterable[Any]) -> Optional[Dict[str, float]]:
    numbers = finite_numbers(values)
    if not numbers:
        return None
    sorted_values = sorted(numbers)
    return {
        "min": sorted_values[0],
        "max": sorted_values[-1],
        "mean": sum(sorted_values) / len(sorted_values),
        "p50": sorted_values[len(sorted_values) // 2],
        "p95": sorted_values[min(len(sorted_values) - 1, int(0.95 * (len(sorted_values) - 1)))],
    }


def topic_summary(dataset: Any) -> Dict[str, Any]:
    data = getattr(dataset, "data", {})
    timestamps = finite_numbers(data.get("timestamp", []))
    summary: Dict[str, Any] = {
        "name": getattr(dataset, "name", "unknown"),
        "multi_id": getattr(dataset, "multi_id", 0),
        "samples": len(timestamps),
    }
    if timestamps:
        summary["start_s"] = timestamps[0] / 1_000_000.0
        summary["end_s"] = timestamps[-1] / 1_000_000.0
        summary["duration_s"] = (timestamps[-1] - timestamps[0]) / 1_000_000.0
    fields: Dict[str, Any] = {}
    for field in WINDOW_FIELDS.get(summary["name"], []):
        if field in data:
            stats = basic_stats(data[field])
            if stats:
                fields[field] = stats
    if fields:
        summary["fields"] = fields
    return summary


def logged_messages(ulog: Any) -> List[Dict[str, Any]]:
    messages = []
    for message in getattr(ulog, "logged_messages", []):
        text = str(getattr(message, "message", ""))
        timestamp = int(getattr(message, "timestamp", 0) or 0)
        level = getattr(message, "log_level_str", None)
        if callable(level):
            level = level()
        messages.append(
            {
                "timestamp_s": timestamp / 1_000_000.0,
                "level": str(level or getattr(message, "log_level", "")),
                "message": text,
            }
        )
    return messages


def relevant_events(messages: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    needles = ("BARO switch", "vertical velocity", "height estimate", "GPS", "EKF", "land", "disarm")
    events = []
    for message in messages:
        text = message.get("message", "")
        if any(needle.lower() in text.lower() for needle in needles):
            events.append(message)
    return events


def state_transitions(dataset: Any, fields: Iterable[str]) -> List[Dict[str, Any]]:
    data = getattr(dataset, "data", {})
    timestamps = as_list(data.get("timestamp", []))
    transitions: List[Dict[str, Any]] = []
    for field in fields:
        values = as_list(data.get(field, []))
        if not timestamps or len(values) != len(timestamps):
            continue
        previous = None
        for timestamp, value in zip(timestamps, values):
            if previous is None:
                previous = value
                continue
            if value != previous:
                transitions.append(
                    {
                        "timestamp_s": float(timestamp) / 1_000_000.0,
                        "field": field,
                        "from": int(previous),
                        "to": int(value),
                    }
                )
                previous = value
    return transitions


def window_stats(datasets: Iterable[Any], event_time_s: float, half_window_s: float) -> Dict[str, Any]:
    start_us = int((event_time_s - half_window_s) * 1_000_000)
    end_us = int((event_time_s + half_window_s) * 1_000_000)
    output: Dict[str, Any] = {}
    for dataset in datasets:
        name = getattr(dataset, "name", "unknown")
        data = getattr(dataset, "data", {})
        timestamps = as_list(data.get("timestamp", []))
        if not timestamps:
            continue
        mask = [start_us <= int(timestamp) <= end_us for timestamp in timestamps]
        if not any(mask):
            continue
        fields: Dict[str, Any] = {}
        for field in WINDOW_FIELDS.get(name, []):
            values = as_list(data.get(field, []))
            if len(values) != len(mask):
                continue
            stats = basic_stats(value for value, keep in zip(values, mask) if keep)
            if stats:
                fields[field] = stats
        if fields:
            output[name] = fields
    return output


def analyze(path: Path, event_window_s: float) -> Dict[str, Any]:
    ulog = load_ulog(path)
    datasets = list(getattr(ulog, "data_list", []))
    summaries = [topic_summary(dataset) for dataset in datasets]
    present = {summary["name"] for summary in summaries}
    messages = logged_messages(ulog)
    events = relevant_events(messages)

    commander = next((dataset for dataset in datasets if getattr(dataset, "name", "") == "commander_state"), None)
    transitions = state_transitions(commander, ("main_state", "arming_state", "nav_state")) if commander else []

    windows = []
    for event in events:
        windows.append(
            {
                "event": event,
                "window_s": event_window_s,
                "topics": window_stats(datasets, float(event["timestamp_s"]), event_window_s),
            }
        )

    return {
        "ulog": str(path),
        "requested_topics": TOPICS,
        "missing_topics": sorted(set(TOPICS) - present),
        "topics": summaries,
        "relevant_logged_messages": events,
        "commander_transitions": transitions,
        "event_windows": windows,
    }


def print_text(result: Dict[str, Any]) -> None:
    print(f"ULog: {result['ulog']}")
    if result["missing_topics"]:
        print("Missing topics: " + ", ".join(result["missing_topics"]))
    else:
        print("All requested topics found.")

    print("\nTopics:")
    for topic in result["topics"]:
        start = topic.get("start_s", 0.0)
        end = topic.get("end_s", 0.0)
        print(f"  {topic['name']}[{topic.get('multi_id', 0)}]: {topic['samples']} samples, {start:.2f}s..{end:.2f}s")

    print("\nRelevant logged messages:")
    if not result["relevant_logged_messages"]:
        print("  none")
    for message in result["relevant_logged_messages"]:
        print(f"  {message['timestamp_s']:.2f}s {message['level']}: {message['message']}")

    print("\nCommander transitions:")
    if not result["commander_transitions"]:
        print("  none detected")
    for transition in result["commander_transitions"]:
        print(
            f"  {transition['timestamp_s']:.2f}s {transition['field']}: "
            f"{transition['from']} -> {transition['to']}"
        )

    if result["event_windows"]:
        print("\nEvent windows:")
        for window in result["event_windows"]:
            event = window["event"]
            print(f"  {event['timestamp_s']:.2f}s {event['message']}")
            for topic, fields in window["topics"].items():
                field_text = ", ".join(f"{field}: max={stats['max']:.3f}" for field, stats in fields.items())
                print(f"    {topic}: {field_text}")


def main() -> int:
    args = parse_args()
    try:
        result = analyze(Path(args.ulog), args.event_window_s)
    except Exception as exc:
        print(f"analyze_ulog_estimator: {exc}", file=sys.stderr)
        return 1

    if args.json_output:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_text(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
