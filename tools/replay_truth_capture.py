#!/usr/bin/env python3
"""Replay XPlaneTruthCapture frames into decoded MAVLink-style sensor rows.

This is an offline first step toward X-Plane-independent bridge tests. It does
not send MAVLink to PX4. It decodes the core deterministic sensor-contract
fields the plugin sends today and writes CSV or JSONL for review/golden-file
comparison. It intentionally does not reproduce stochastic sensor noise.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import sys
import zipfile
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, TextIO


GRAVITY_M_S2 = 9.80665
KNOT_TO_MPS = 0.514444
AIR_DENSITY_SEA_LEVEL = 1.225

DEFAULT_PERIODS = {
    "sensor": 1.0 / 200.0,
    "gps": 1.0 / 20.0,
    "state": 1.0 / 10.0,
    "rc": 1.0 / 10.0,
}

CSV_FIELDS = [
    "message",
    "frame_id",
    "sim_time_s",
    "time_usec",
    "lat",
    "lon",
    "alt_mm",
    "xacc",
    "yacc",
    "zacc",
    "xgyro",
    "ygyro",
    "zgyro",
    "abs_pressure_hpa",
    "diff_pressure_hpa",
    "pressure_alt",
    "temperature_c",
    "vn_cms",
    "ve_cms",
    "vd_cms",
    "vel_cms",
    "cog_cdeg",
    "yaw_cdeg",
    "eph_cm",
    "epv_cm",
    "satellites_visible",
    "fix_type",
    "gps_id",
    "q0",
    "q1",
    "q2",
    "q3",
    "rollspeed",
    "pitchspeed",
    "yawspeed",
    "ind_airspeed_cms",
    "true_airspeed_cms",
    "chan1_raw",
    "chan2_raw",
    "chan3_raw",
    "chan4_raw",
    "rssi",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", help="XPlaneTruthCapture run folder or zip file")
    parser.add_argument("--output", "-o", help="Output path. Defaults to stdout")
    parser.add_argument("--format", choices=("csv", "jsonl"), default="csv")
    parser.add_argument(
        "--messages",
        default="sensor,gps,state,rc",
        help="Comma-separated message groups to decode: sensor,gps,state,rc",
    )
    parser.add_argument("--sensor-rate-hz", type=float, default=200.0)
    parser.add_argument("--gps-rate-hz", type=float, default=20.0)
    parser.add_argument("--state-rate-hz", type=float, default=10.0)
    parser.add_argument("--rc-rate-hz", type=float, default=10.0)
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames")
    parser.add_argument("--max-messages", type=int, default=0, help="Stop after N emitted messages")
    return parser.parse_args()


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
            raw = self._zip.open(candidates[0], "r")
            self._text = io.TextIOWrapper(raw, encoding="utf-8", newline="")
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
            self._text = None
        if self._zip is not None:
            self._zip.close()
            self._zip = None


def safe_float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def safe_int(row: Dict[str, str], key: str, default: int = 0) -> int:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def parse_float_array(value: str) -> List[float]:
    if not value:
        return []
    result: List[float] = []
    for item in value.split(";"):
        if item == "":
            continue
        try:
            result.append(float(item))
        except ValueError:
            result.append(0.0)
    return result


def clamp_i16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def clamp_u16(value: float) -> int:
    return max(0, min(65535, int(round(value))))


def wrap_degrees_360(degrees: float) -> float:
    return degrees % 360.0


def degrees_to_centidegrees(degrees: float) -> int:
    value = clamp_u16(wrap_degrees_360(degrees) * 100.0)
    return 0 if value == 36000 else value


def pressure_from_altitude(altitude_m: float) -> float:
    p0 = 1013.25
    t0 = 288.15
    lapse = 0.0065
    gas_constant = 8.3144598
    gravity = 9.80665
    molar_mass = 0.0289644
    return p0 * pow(1.0 - (lapse * altitude_m) / t0, (gravity * molar_mass) / (gas_constant * lapse))


def local_ned_velocity(row: Dict[str, str]) -> tuple[float, float, float]:
    vn = -safe_float(row, "sim/flightmodel/position/local_vz")
    ve = safe_float(row, "sim/flightmodel/position/local_vx")
    vd = -safe_float(row, "sim/flightmodel/position/local_vy")
    return vn, ve, vd


def true_course_from_velocity(vn: float, ve: float) -> float:
    return wrap_degrees_360(math.degrees(math.atan2(ve, vn)))


def base_row(message: str, row: Dict[str, str], timestamp_usec: int) -> Dict[str, object]:
    return {
        "message": message,
        "frame_id": safe_int(row, "frame_id"),
        "sim_time_s": safe_float(row, "sim/time/total_flight_time_sec"),
        "time_usec": timestamp_usec,
    }


def decode_sensor(row: Dict[str, str], timestamp_usec: int) -> Dict[str, object]:
    decoded = base_row("HIL_SENSOR", row, timestamp_usec)
    ias_mps = safe_float(row, "sim/flightmodel/position/indicated_airspeed") * KNOT_TO_MPS
    dynamic_pressure_pa = 0.5 * AIR_DENSITY_SEA_LEVEL * ias_mps * ias_mps
    altitude_m = safe_float(row, "sim/flightmodel/position/elevation")
    decoded.update(
        {
            "xacc": -safe_float(row, "sim/flightmodel/forces/g_axil") * GRAVITY_M_S2,
            "yacc": safe_float(row, "sim/flightmodel/forces/g_side") * GRAVITY_M_S2,
            "zacc": -safe_float(row, "sim/flightmodel/forces/g_nrml") * GRAVITY_M_S2,
            "xgyro": safe_float(row, "sim/flightmodel/position/Prad"),
            "ygyro": safe_float(row, "sim/flightmodel/position/Qrad"),
            "zgyro": safe_float(row, "sim/flightmodel/position/Rrad"),
            "abs_pressure_hpa": pressure_from_altitude(altitude_m),
            "diff_pressure_hpa": dynamic_pressure_pa * 0.01,
            "pressure_alt": 0.0,
            "temperature_c": safe_float(row, "sim/cockpit2/temperature/outside_air_temp_degc"),
        }
    )
    return decoded


def decode_gps(row: Dict[str, str], timestamp_usec: int, last_cog: int) -> tuple[Dict[str, object], int]:
    vn, ve, vd = local_ned_velocity(row)
    horizontal_speed = math.hypot(vn, ve)
    cog = last_cog
    if horizontal_speed >= 0.5:
        cog = degrees_to_centidegrees(true_course_from_velocity(vn, ve))

    yaw = degrees_to_centidegrees(safe_float(row, "sim/flightmodel/position/psi"))
    decoded = base_row("HIL_GPS", row, timestamp_usec)
    decoded.update(
        {
            "lat": int(safe_float(row, "sim/flightmodel/position/latitude") * 1e7),
            "lon": int(safe_float(row, "sim/flightmodel/position/longitude") * 1e7),
            "alt_mm": int(safe_float(row, "sim/flightmodel/position/elevation") * 1000.0),
            "vn_cms": clamp_i16(vn * 100.0),
            "ve_cms": clamp_i16(ve * 100.0),
            "vd_cms": clamp_i16(vd * 100.0),
            "vel_cms": clamp_u16(horizontal_speed * 100.0),
            "cog_cdeg": cog,
            "yaw_cdeg": 35999 if yaw == 0 else yaw,
            "eph_cm": 80,
            "epv_cm": 100,
            "satellites_visible": 16,
            "fix_type": 3,
            "gps_id": 0,
        }
    )
    return decoded, cog


def decode_state(row: Dict[str, str], timestamp_usec: int) -> Dict[str, object]:
    vn, ve, vd = local_ned_velocity(row)
    accel_x = -safe_float(row, "sim/flightmodel/forces/g_axil") * GRAVITY_M_S2
    accel_y = safe_float(row, "sim/flightmodel/forces/g_side") * GRAVITY_M_S2
    accel_z = -safe_float(row, "sim/flightmodel/forces/g_nrml") * GRAVITY_M_S2
    q = parse_float_array(row.get("sim/flightmodel/position/q", ""))[:4]
    q.extend([0.0] * (4 - len(q)))

    decoded = base_row("HIL_STATE_QUATERNION", row, timestamp_usec)
    decoded.update(
        {
            "lat": int(safe_float(row, "sim/flightmodel/position/latitude") * 1e7),
            "lon": int(safe_float(row, "sim/flightmodel/position/longitude") * 1e7),
            "alt_mm": int(safe_float(row, "sim/flightmodel/position/elevation") * 1000.0),
            "vn_cms": clamp_i16(vn * 100.0),
            "ve_cms": clamp_i16(ve * 100.0),
            "vd_cms": clamp_i16(vd * 100.0),
            "q0": q[0],
            "q1": q[1],
            "q2": q[2],
            "q3": q[3],
            "rollspeed": safe_float(row, "sim/flightmodel/position/Prad"),
            "pitchspeed": safe_float(row, "sim/flightmodel/position/Qrad"),
            "yawspeed": safe_float(row, "sim/flightmodel/position/Rrad"),
            "ind_airspeed_cms": clamp_u16(safe_float(row, "sim/flightmodel/position/indicated_airspeed") * KNOT_TO_MPS * 100.0),
            "true_airspeed_cms": clamp_u16(safe_float(row, "sim/flightmodel/position/true_airspeed") * 100.0),
            "xacc": clamp_i16((accel_x / GRAVITY_M_S2) * 1000.0),
            "yacc": clamp_i16((accel_y / GRAVITY_M_S2) * 1000.0),
            "zacc": clamp_i16((accel_z / GRAVITY_M_S2) * 1000.0),
        }
    )
    return decoded


def map_rc(value: float, min_value: float, max_value: float) -> int:
    mapped = 1000.0 + ((value - min_value) / (max_value - min_value)) * 1000.0
    return clamp_u16(mapped)


def decode_rc(row: Dict[str, str], timestamp_usec: int) -> Dict[str, object]:
    decoded = base_row("HIL_RC_INPUTS_RAW", row, timestamp_usec)
    decoded.update(
        {
            "chan1_raw": map_rc(safe_float(row, "sim/joystick/yoke_roll_ratio"), -1.0, 1.0),
            "chan2_raw": map_rc(safe_float(row, "sim/joystick/yoke_pitch_ratio"), -1.0, 1.0),
            "chan3_raw": map_rc(safe_float(row, "sim/cockpit2/engine/actuators/throttle_ratio_all"), 0.0, 1.0),
            "chan4_raw": map_rc(safe_float(row, "sim/joystick/yoke_heading_ratio"), -1.0, 1.0),
            "rssi": 255,
        }
    )
    return decoded


def periods_from_args(args: argparse.Namespace) -> Dict[str, float]:
    return {
        "sensor": 1.0 / args.sensor_rate_hz,
        "gps": 1.0 / args.gps_rate_hz,
        "state": 1.0 / args.state_rate_hz,
        "rc": 1.0 / args.rc_rate_hz,
    }


def iter_decoded_rows(args: argparse.Namespace) -> Iterator[Dict[str, object]]:
    selected = {item.strip().lower() for item in args.messages.split(",") if item.strip()}
    unknown = selected - set(DEFAULT_PERIODS)
    if unknown:
        raise ValueError(f"Unknown message group(s): {', '.join(sorted(unknown))}")

    periods = periods_from_args(args)
    last_send = {name: 0.0 for name in DEFAULT_PERIODS}
    start_sim_time: Optional[float] = None
    last_cog = 0
    emitted = 0

    with CaptureFrames(Path(args.capture)) as frames:
        reader = csv.DictReader(frames)
        for frame_index, row in enumerate(reader, start=1):
            if args.max_frames and frame_index > args.max_frames:
                return

            sim_time = safe_float(row, "sim/time/total_flight_time_sec")
            if start_sim_time is None:
                start_sim_time = sim_time
            timestamp_usec = 1_000_000 + int(round(max(0.0, sim_time - start_sim_time) * 1_000_000.0))

            if "sensor" in selected and sim_time - last_send["sensor"] >= periods["sensor"]:
                yield decode_sensor(row, timestamp_usec)
                emitted += 1
                last_send["sensor"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return

            if "gps" in selected and sim_time - last_send["gps"] >= periods["gps"]:
                decoded, last_cog = decode_gps(row, timestamp_usec, last_cog)
                yield decoded
                emitted += 1
                last_send["gps"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return

            if "state" in selected and sim_time - last_send["state"] >= periods["state"]:
                yield decode_state(row, timestamp_usec)
                emitted += 1
                last_send["state"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return

            if "rc" in selected and sim_time - last_send["rc"] >= periods["rc"]:
                yield decode_rc(row, timestamp_usec)
                emitted += 1
                last_send["rc"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return


def write_rows(rows: Iterable[Dict[str, object]], output: TextIO, output_format: str) -> None:
    if output_format == "jsonl":
        for row in rows:
            output.write(json.dumps(row, sort_keys=True) + "\n")
        return

    writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction="ignore")
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main() -> int:
    args = parse_args()
    try:
        rows = iter_decoded_rows(args)
        if args.output:
            with Path(args.output).open("w", encoding="utf-8", newline="") as output:
                write_rows(rows, output, args.format)
        else:
            write_rows(rows, sys.stdout, args.format)
    except Exception as exc:
        print(f"replay_truth_capture: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
