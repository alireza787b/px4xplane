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
    parser.add_argument(
        "--disable-ground-stationary-contract",
        action="store_true",
        help="Do not apply the stationary-ground sensor contract used by the bridge.",
    )
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


def normalize_quaternion(q: List[float]) -> List[float]:
    q = q[:4]
    q.extend([0.0] * (4 - len(q)))
    norm = math.sqrt(sum(item * item for item in q))
    if not math.isfinite(norm) or norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [item / norm for item in q]


def rotate_world_to_body(q: List[float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Apply R(q)^T to a vector, matching the bridge stationary accel path."""
    w, x, y, z = normalize_quaternion(q)
    vx, vy, vz = vector
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return (
        r00 * vx + r10 * vy + r20 * vz,
        r01 * vx + r11 * vy + r21 * vz,
        r02 * vx + r12 * vy + r22 * vz,
    )


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


def stationary_baro_liveness_noise_m(timestamp_usec: int) -> float:
    # Deterministic replay equivalent of the bridge's tiny live baro noise.
    # Keeps stationary pressure samples from becoming exactly identical.
    return 0.003 * math.sin(timestamp_usec * 0.000071)


def signed_dynamic_pressure_hpa_from_ias_knots(ias_knots: float) -> float:
    ias_mps = ias_knots * KNOT_TO_MPS
    dynamic_pressure_pa = 0.5 * AIR_DENSITY_SEA_LEVEL * ias_mps * abs(ias_mps)
    return dynamic_pressure_pa * 0.01


def local_ned_velocity(row: Dict[str, str]) -> tuple[float, float, float]:
    vn = -safe_float(row, "sim/flightmodel/position/local_vz")
    ve = safe_float(row, "sim/flightmodel/position/local_vx")
    vd = -safe_float(row, "sim/flightmodel/position/local_vy")
    return vn, ve, vd


def horizontal_speed(row: Dict[str, str]) -> float:
    vn, ve, _ = local_ned_velocity(row)
    return math.hypot(vn, ve)


def mapped_motor_command_active(row: Dict[str, str]) -> bool:
    commands = parse_float_array(row.get("sim/flightmodel/engine/ENGN_thro_use", ""))
    return any(command > 0.08 for command in commands)


class GroundStationaryContract:
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.active = False
        self.initialized = False
        self.last_sim_time_s = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0
        self.elevation_m = 0.0

    def update(self, row: Dict[str, str]) -> "GroundStationaryContract":
        sim_time_s = safe_float(row, "sim/time/total_flight_time_sec")
        if not self.enabled:
            self.active = False
            self.initialized = True
            self.last_sim_time_s = sim_time_s
            return self

        if self.initialized and sim_time_s < self.last_sim_time_s - 0.001:
            self.active = False

        if self.initialized and abs(sim_time_s - self.last_sim_time_s) < 1e-7:
            return self

        self.initialized = True
        self.last_sim_time_s = sim_time_s

        motor_active = mapped_motor_command_active(row)
        if not self.active and self._is_stationary(row, agl_max=0.45, horizontal_max=6.0) and not motor_active:
            self.active = True
            self.latitude_deg = safe_float(row, "sim/flightmodel/position/latitude")
            self.longitude_deg = safe_float(row, "sim/flightmodel/position/longitude")
            self.elevation_m = safe_float(row, "sim/flightmodel/position/elevation")
        elif self.active and (not self._is_stationary(row, agl_max=0.70, horizontal_max=8.0) or motor_active):
            self.active = False

        return self

    @staticmethod
    def _is_stationary(row: Dict[str, str], agl_max: float, horizontal_max: float) -> bool:
        agl_m = safe_float(row, "sim/flightmodel/position/y_agl", default=999.0)
        return (
            safe_int(row, "sim/flightmodel/failures/onground_any") != 0
            and -0.75 <= agl_m < agl_max
            and horizontal_speed(row) < horizontal_max
        )


def sensor_position(row: Dict[str, str], contract: Optional[GroundStationaryContract]) -> tuple[float, float, float]:
    if contract is not None and contract.active:
        return contract.latitude_deg, contract.longitude_deg, contract.elevation_m
    return (
        safe_float(row, "sim/flightmodel/position/latitude"),
        safe_float(row, "sim/flightmodel/position/longitude"),
        safe_float(row, "sim/flightmodel/position/elevation"),
    )


def sensor_ned_velocity(
    row: Dict[str, str], contract: Optional[GroundStationaryContract]
) -> tuple[float, float, float]:
    if contract is not None and contract.active:
        return 0.0, 0.0, 0.0
    return local_ned_velocity(row)


def sensor_accel(
    row: Dict[str, str], contract: Optional[GroundStationaryContract]
) -> tuple[float, float, float]:
    if contract is not None and contract.active:
        q = parse_float_array(row.get("sim/flightmodel/position/q", ""))
        return rotate_world_to_body(q, (0.0, 0.0, -GRAVITY_M_S2))
    return (
        -safe_float(row, "sim/flightmodel/forces/g_axil") * GRAVITY_M_S2,
        safe_float(row, "sim/flightmodel/forces/g_side") * GRAVITY_M_S2,
        -safe_float(row, "sim/flightmodel/forces/g_nrml") * GRAVITY_M_S2,
    )


def true_course_from_velocity(vn: float, ve: float) -> float:
    return wrap_degrees_360(math.degrees(math.atan2(ve, vn)))


def base_row(message: str, row: Dict[str, str], timestamp_usec: int) -> Dict[str, object]:
    return {
        "message": message,
        "frame_id": safe_int(row, "frame_id"),
        "sim_time_s": safe_float(row, "sim/time/total_flight_time_sec"),
        "time_usec": timestamp_usec,
    }


def decode_sensor(
    row: Dict[str, str],
    timestamp_usec: int,
    contract: Optional[GroundStationaryContract] = None,
) -> Dict[str, object]:
    decoded = base_row("HIL_SENSOR", row, timestamp_usec)
    _, _, altitude_m = sensor_position(row, contract)
    xacc, yacc, zacc = sensor_accel(row, contract)
    stationary_ground = contract is not None and contract.active
    baro_altitude_m = altitude_m
    if stationary_ground:
        baro_altitude_m += stationary_baro_liveness_noise_m(timestamp_usec)
    decoded.update(
        {
            "xacc": xacc,
            "yacc": yacc,
            "zacc": zacc,
            "xgyro": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Prad"),
            "ygyro": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Qrad"),
            "zgyro": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Rrad"),
            "abs_pressure_hpa": pressure_from_altitude(baro_altitude_m),
            "diff_pressure_hpa": signed_dynamic_pressure_hpa_from_ias_knots(
                safe_float(row, "sim/flightmodel/position/indicated_airspeed")
            ),
            "pressure_alt": 0.0,
            "temperature_c": safe_float(row, "sim/cockpit2/temperature/outside_air_temp_degc"),
        }
    )
    return decoded


def decode_gps(
    row: Dict[str, str],
    timestamp_usec: int,
    last_cog: int,
    contract: Optional[GroundStationaryContract] = None,
) -> tuple[Dict[str, object], int]:
    vn, ve, vd = sensor_ned_velocity(row, contract)
    horizontal_speed = math.hypot(vn, ve)
    cog = last_cog
    if horizontal_speed >= 0.5:
        cog = degrees_to_centidegrees(true_course_from_velocity(vn, ve))

    yaw = degrees_to_centidegrees(safe_float(row, "sim/flightmodel/position/psi"))
    lat, lon, alt_m = sensor_position(row, contract)
    decoded = base_row("HIL_GPS", row, timestamp_usec)
    decoded.update(
        {
            "lat": int(lat * 1e7),
            "lon": int(lon * 1e7),
            "alt_mm": int(alt_m * 1000.0),
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


def decode_state(
    row: Dict[str, str],
    timestamp_usec: int,
    contract: Optional[GroundStationaryContract] = None,
) -> Dict[str, object]:
    vn, ve, vd = sensor_ned_velocity(row, contract)
    accel_x, accel_y, accel_z = sensor_accel(row, contract)
    q = parse_float_array(row.get("sim/flightmodel/position/q", ""))[:4]
    q.extend([0.0] * (4 - len(q)))
    stationary_ground = contract is not None and contract.active
    lat, lon, alt_m = sensor_position(row, contract)

    decoded = base_row("HIL_STATE_QUATERNION", row, timestamp_usec)
    decoded.update(
        {
            "lat": int(lat * 1e7),
            "lon": int(lon * 1e7),
            "alt_mm": int(alt_m * 1000.0),
            "vn_cms": clamp_i16(vn * 100.0),
            "ve_cms": clamp_i16(ve * 100.0),
            "vd_cms": clamp_i16(vd * 100.0),
            "q0": q[0],
            "q1": q[1],
            "q2": q[2],
            "q3": q[3],
            "rollspeed": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Prad"),
            "pitchspeed": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Qrad"),
            "yawspeed": 0.0 if stationary_ground else safe_float(row, "sim/flightmodel/position/Rrad"),
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
    ground_contract = GroundStationaryContract(
        enabled=not getattr(args, "disable_ground_stationary_contract", False)
    )

    with CaptureFrames(Path(args.capture)) as frames:
        reader = csv.DictReader(frames)
        for frame_index, row in enumerate(reader, start=1):
            if args.max_frames and frame_index > args.max_frames:
                return

            sim_time = safe_float(row, "sim/time/total_flight_time_sec")
            if start_sim_time is None:
                start_sim_time = sim_time
            timestamp_usec = 1_000_000 + int(round(max(0.0, sim_time - start_sim_time) * 1_000_000.0))
            ground_contract.update(row)

            if "sensor" in selected and sim_time - last_send["sensor"] >= periods["sensor"]:
                yield decode_sensor(row, timestamp_usec, ground_contract)
                emitted += 1
                last_send["sensor"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return

            if "gps" in selected and sim_time - last_send["gps"] >= periods["gps"]:
                decoded, last_cog = decode_gps(row, timestamp_usec, last_cog, ground_contract)
                yield decoded
                emitted += 1
                last_send["gps"] = sim_time
            if args.max_messages and emitted >= args.max_messages:
                return

            if "state" in selected and sim_time - last_send["state"] >= periods["state"]:
                yield decode_state(row, timestamp_usec, ground_contract)
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
